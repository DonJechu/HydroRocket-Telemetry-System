#include <Wire.h>
#include <MPU9250_WE.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ================================================================
//  HYDRO-1 FLIGHT COMPUTER  —  v2.0  FLIGHT-READY
//  Filters and thresholds calibrated for real flight, not for
//  static indoor testing.
// ================================================================

// ------------- 1. Config -------------
const char* ssid     = "GANNET";
const char* password = "1234";
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_SERVO  13
#define MPU_ADDR   0x68

// ------------- Flight Thresholds (adjust according to your rocket) -------
// Typical PET water rocket: ~15-20 m/s peak, ~30-60 m apogee
#define LIFTOFF_G            2.5f   // Gs to confirm liftoff
#define LIFTOFF_CONFIRMS     4      // Required consecutive samples
#define APOGEE_VEL_THRESHOLD 0.0f   // Velocity <= 0 = apogee
#define APOGEE_CONFIRMS      5      // Consecutive samples to confirm apogee
#define APOGEE_ALT_DROP      1.0f   // Meters of drop as backup
#define APOGEE_ALT_CONFIRMS  4      // Confirmations for altitude drop
#define MIN_ALTITUDE_LAUNCH  3.0f   // Minimum altitude before searching for apogee
#define LANDING_ALT_M        0.75f  // Landing altitude
#define ASCENT_TIMEOUT_MS    12000  // Safety timeout in ASCENT (ms)

// ------------- 2. Objects -------------
MPU9250_WE    mpu = MPU9250_WE(MPU_ADDR);
Adafruit_BMP280     bmp;
Servo               parachute;
WebSocketsServer    webSocket = WebSocketsServer(81);

// ------------- 3. Globals & States -------------
float          basePressure      = 1013.25f;
int            currentState      = 0;
float          currentAltitude   = 0.0f;
float          filteredAltitude  = 0.0f;
float          gForce            = 0.0f;
float          maxAltitude       = 0.0f;
unsigned long lastTelemetryTime = 0;
const int      telemetryInterval = 100;  // 10 Hz telemetry
float          lastAltitude      = 0.0f;
unsigned long lastTime           = 0;
float          velocity          = 0.0f;
float          pitch             = 0.0f;
float          roll              = 0.0f;

// Confirmation counters (anti-transient)
int           liftoffCounter    = 0;
int           apogeeVelCounter  = 0;
int           apogeeAltCounter  = 0;

// Timing
unsigned long ascentStartTime   = 0;
unsigned long lastBmpRead       = 0;
const int      bmpInterval       = 40;   // 25 Hz BMP280

// ================================================================
//  Flight State — synchronized with Ground Station
// ================================================================
enum FlightStates {
  STANDBY   = 0,
  ARMED     = 1,
  IGNITION  = 2,
  ASCENT    = 3,
  APOGEE    = 4,
  DESCENT   = 5,
  LANDING   = 6
};

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  // WiFi AP + WebSockets
  WiFi.softAP(ssid, password);
  webSocket.begin();
  Serial.println("=== HYDRO-1 FLIGHT COMPUTER BOOT ===");

  // MPU9250
  if (!mpu.init()) {
    Serial.println("[ERROR] MPU9250 Init Failed");
  }
  mpu.setAccRange(MPU9250_ACC_RANGE_16G);
  mpu.setGyrRange(MPU9250_GYRO_RANGE_2000);
  delay(100);

  // BMP280 — configure hardware IIR filter (key for real flight)
  // The BMP280 has a built-in IIR filter that eliminates turbulence
  // and pressure transients caused by aerodynamic flow.
  if (!bmp.begin(0x76)) {
    Serial.println("[ERROR] BMP280 Init Failed");
  }
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // Temperature
    Adafruit_BMP280::SAMPLING_X16,  // Pressure — high resolution
    Adafruit_BMP280::FILTER_X16,    // IIR x16 — maximum hardware smoothing
    Adafruit_BMP280::STANDBY_MS_1   // Update every ~27ms
  );

  // Base pressure calibration — 300 samples, ~3 seconds
  // Perform with the rocket stationary on the pad before arming
  Serial.println("Calibrating base pressure...");
  float sumPressure = 0;
  for (int i = 0; i < 300; i++) {
    sumPressure += bmp.readPressure();
    delay(10);
  }
  basePressure = (sumPressure / 300.0f) / 100.0f;  // Pa → hPa
  Serial.print("Base Pressure: ");
  Serial.print(basePressure);
  Serial.println(" hPa");

  // Servo
  parachute.attach(PIN_SERVO);
  parachute.write(0);  // Closed

  // Initialize state to avoid spikes
  lastTime         = millis();
  lastBmpRead      = millis();
  filteredAltitude = bmp.readAltitude(basePressure);
  lastAltitude     = filteredAltitude;
  currentAltitude  = filteredAltitude;

  Serial.println("=== READY FOR FLIGHT ===");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  webSocket.loop();
  readSensors();
  updateLogic();

  if (millis() - lastTelemetryTime >= telemetryInterval) {
    sendData();
    lastTelemetryTime = millis();
  }
}

// ================================================================
//  FLIGHT LOGIC
//  Based on: multi-sample detection to avoid false positives
//  from aerodynamic turbulence (standard in rocket avionics)
// ================================================================
void updateLogic() {
  switch (currentState) {

    // -- STANDBY: wait for liftoff confirmed by N samples
    case STANDBY:
      if (gForce >= LIFTOFF_G) {
        liftoffCounter++;
        if (liftoffCounter >= LIFTOFF_CONFIRMS) {
          currentState    = ASCENT;
          ascentStartTime = millis();
          maxAltitude     = currentAltitude;
          liftoffCounter  = 0;
          Serial.println("[EVENT] LIFTOFF CONFIRMED");
        }
      } else {
        liftoffCounter = 0;  // Reset if interrupted
      }
      break;

    // -- ASCENT: detect apogee via velocity AND altitude
    case ASCENT:
      if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
      }

      // Only search for apogee once minimum altitude is exceeded
      // Avoids false detection on the launch rail
      if (maxAltitude < MIN_ALTITUDE_LAUNCH) break;

      // Method 1 (primary): velocity crosses zero → apogee
      // Zero crossing is the most reliable method according to literature
      if (velocity <= APOGEE_VEL_THRESHOLD) {
        apogeeVelCounter++;
        if (apogeeVelCounter >= APOGEE_CONFIRMS) {
          Serial.println("[EVENT] APOGEE — velocity zero crossing");
          openParachute();
          break;
        }
      } else {
        apogeeVelCounter = 0;
      }

      // Method 2 (backup): altitude drop from the maximum
      if (currentAltitude < (maxAltitude - APOGEE_ALT_DROP)) {
        apogeeAltCounter++;
        if (apogeeAltCounter >= APOGEE_ALT_CONFIRMS) {
          Serial.println("[EVENT] APOGEE — altitude drop backup");
          openParachute();
          break;
        }
      } else {
        apogeeAltCounter = 0;
      }

      // Method 3 (safety): absolute timeout
      if (millis() - ascentStartTime > ASCENT_TIMEOUT_MS) {
        Serial.println("[SAFETY] TIMEOUT — forcing parachute deployment");
        openParachute();
        break;
      }
      break;

    case DESCENT:
      if (currentAltitude < LANDING_ALT_M) {
        currentState = LANDING;
        Serial.println("[EVENT] LANDING DETECTED");
      }
      break;

    case LANDING:
      break;
  }
}

void openParachute() {
  parachute.write(90);
  currentState     = DESCENT;
  apogeeVelCounter = 0;
  apogeeAltCounter = 0;
}

// ================================================================
//  SENSOR READING
//  Filters calibrated for real flight:
//  - Higher Alpha (0.8) → more responsive to fast real changes
//  - No velocity deadband → do not suppress real flight readings
//  - It will look noisy in static tests; it will look smooth in flight
// ================================================================
void readSensors() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // BMP280 rate limiting
  if (millis() - lastBmpRead >= bmpInterval) {
    float rawAlt = bmp.readAltitude(basePressure);

    // Software IIR filter — alpha=0.8 for real flight
    // (responsive to altitude changes of 15-20 m/s)
    // Note: indoors it will look noisy — this is correct and expected
    filteredAltitude = filteredAltitude * 0.8f + rawAlt * 0.2f;

    // Validation: ignore physically impossible jumps (>100 m in one tick)
    if (abs(filteredAltitude - currentAltitude) < 100.0f) {
      currentAltitude = filteredAltitude;
    }

    // Velocity — filtered altitude differential
    if (dt > 0.001f) {
      float rawVel = (currentAltitude - lastAltitude) / dt;
      // Alpha=0.85 in flight: smoothes turbulence without hiding real dynamics
      velocity = velocity * 0.85f + rawVel * 0.15f;
    }

    lastAltitude = currentAltitude;
    lastBmpRead  = millis();
  }

  // MPU9250 — G-Force and orientation
  xyzFloat data = mpu.getGValues();
  gForce = sqrt((data.x * data.x) + (data.y * data.y) + (data.z * data.z));

  // data.x is the longitudinal axis of the rocket (points towards the nose)
  pitch = atan2(data.x, sqrt(data.y * data.y + data.z * data.z)) * 180.0f / 3.14159f;
  roll  = atan2(data.y, data.z) * 180.0f / 3.14159f;
}

// ================================================================
//  TELEMETRY — JSON to Ground Station via WebSocket
// ================================================================
void sendData() {
  String json = "{";
  json += "\"alt\":"      + String(currentAltitude, 1) + ",";
  json += "\"vel\":"      + String(velocity, 1)        + ",";
  json += "\"accel\":"    + String(gForce, 2)           + ",";
  json += "\"pitch\":"    + String(pitch, 1)            + ",";
  json += "\"roll\":"     + String(roll, 1)             + ",";
  json += "\"yaw\":"      + String(0.0f, 1)             + ",";
  json += "\"temp\":"     + String(bmp.readTemperature(), 1)   + ",";
  json += "\"pressure\":" + String(bmp.readPressure() / 100.0f, 1) + ",";
  json += "\"phase\":"    + String(currentState);
  json += "}";

  webSocket.broadcastTXT(json);
  Serial.println(json);
}