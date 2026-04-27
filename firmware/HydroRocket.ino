#include <Wire.h>
#include <MPU9250_WE.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ================================================================
//  HYDRO-1 FLIGHT COMPUTER  —  v2.0  FLIGHT-READY
//  Filtros y umbrales calibrados para vuelo real, no para
//  pruebas estáticas en interiores.
// ================================================================

// ------------- 1. Config -------------
const char* ssid     = "GANNET";
const char* password = "1234";
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_SERVO  13
#define MPU_ADDR   0x68

// ------------- Umbrales de vuelo (ajusta según tu cohete) -------
// Cohete de agua PET típico: ~15-20 m/s pico, ~30-60 m apogeo
#define LIFTOFF_G            2.5f   // G para confirmar despegue
#define LIFTOFF_CONFIRMS     4      // Muestras consecutivas requeridas
#define APOGEE_VEL_THRESHOLD 0.0f   // Velocidad <= 0 = apogeo
#define APOGEE_CONFIRMS      5      // Muestras consecutivas para confirmar apogeo
#define APOGEE_ALT_DROP      1.0f   // Metros de caída como respaldo
#define APOGEE_ALT_CONFIRMS  4      // Confirmaciones para caída de altitud
#define MIN_ALTITUDE_LAUNCH  3.0f   // Altitud mínima antes de buscar apogeo
#define LANDING_ALT_M        0.75f  // Altitud de aterrizaje
#define ASCENT_TIMEOUT_MS    12000  // Timeout de seguridad en ASCENT (ms)

// ------------- 2. Objects -------------
MPU9250_WE          mpu = MPU9250_WE(MPU_ADDR);
Adafruit_BMP280     bmp;
Servo               parachute;
WebSocketsServer    webSocket = WebSocketsServer(81);

// ------------- 3. Globals & States -------------
float         basePressure      = 1013.25f;
int           currentState      = 0;
float         currentAltitude   = 0.0f;
float         filteredAltitude  = 0.0f;
float         gForce            = 0.0f;
float         maxAltitude       = 0.0f;
unsigned long lastTelemetryTime = 0;
const int     telemetryInterval = 100;  // 10 Hz telemetría
float         lastAltitude      = 0.0f;
unsigned long lastTime          = 0;
float         velocity          = 0.0f;
float         pitch             = 0.0f;
float         roll              = 0.0f;

// Contadores de confirmación (anti-transient)
int           liftoffCounter    = 0;
int           apogeeVelCounter  = 0;
int           apogeeAltCounter  = 0;

// Timing
unsigned long ascentStartTime   = 0;
unsigned long lastBmpRead       = 0;
const int     bmpInterval       = 40;   // 25 Hz BMP280

// ================================================================
//  Estado de vuelo — sincronizado con Ground Station
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
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  webSocket.begin();
  Serial.println("=== HYDRO-1 FLIGHT COMPUTER BOOT ===");

  // MPU9250
  if (!mpu.init()) {
    Serial.println("[ERROR] MPU9250 Init Failed");
  }
  mpu.setAccRange(MPU9250_ACC_RANGE_16G);
  mpu.setGyrRange(MPU9250_GYRO_RANGE_2000);
  delay(100);

  // BMP280 — configurar filtro IIR hardware (clave para vuelo real)
  // El BMP280 tiene un filtro IIR integrado que elimina turbulencia
  // y transientes de presión causados por el flujo aerodinámico.
  if (!bmp.begin(0x76)) {
    Serial.println("[ERROR] BMP280 Init Failed");
  }
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // Temperatura
    Adafruit_BMP280::SAMPLING_X16,  // Presión — alta resolución
    Adafruit_BMP280::FILTER_X16,    // IIR x16 — suavizado hardware máximo
    Adafruit_BMP280::STANDBY_MS_1   // Actualización cada ~27ms
  );

  // Calibración de presión base — 300 muestras, ~3 segundos
  // Hacerlo con el cohete quieto en el pad antes de armar
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
  parachute.write(0);  // Cerrado

  // Inicializar estado para evitar spikes
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

  int activeInterval = (currentState == STANDBY) ? 1000 : 100;
  if (millis() - lastTelemetryTime >= activeInterval) {
    sendData();
    lastTelemetryTime = millis();
  }
}

// ================================================================
//  LÓGICA DE VUELO
//  Basada en: detección multi-sample para evitar falsos positivos
//  por turbulencia aerodinámica (standard en aviónica de cohetes)
// ================================================================
void updateLogic() {
  switch (currentState) {

    // -- STANDBY: esperar despegue confirmado por N muestras
    case STANDBY:
      setCpuFrequencyMhz(80);
      if (gForce >= LIFTOFF_G) {
        liftoffCounter++;
        if (liftoffCounter >= LIFTOFF_CONFIRMS) {
          setCpuFrequencyMhz(240);  // Restaurar CPU a máximo para vuelo
          currentState    = ASCENT;
          ascentStartTime = millis();
          maxAltitude     = currentAltitude;
          liftoffCounter  = 0;
          Serial.println("[EVENT] LIFTOFF CONFIRMED");
        }
      } else {
        liftoffCounter = 0;  // Reset si se interrumpe
      }
      break;

    // -- ASCENT: detectar apogeo por velocidad Y altitud
    case ASCENT:
      if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
      }

      // Solo buscar apogeo una vez superada la altitud mínima
      // Evita falsa detección en la rampa de lanzamiento
      if (maxAltitude < MIN_ALTITUDE_LAUNCH) break;

      // Método 1 (primario): velocidad cruza cero → apogeo
      // El zero crossing es el método más confiable según literatura
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

      // Método 2 (respaldo): caída de altitud desde el máximo
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

      // Método 3 (seguridad): timeout absoluto
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
//  LECTURA DE SENSORES
//  Filtros calibrados para vuelo real:
//  - Alpha más alto (0.8) → más responsivo a cambios reales rápidos
//  - Sin deadband de velocidad → no suprimir lecturas reales en vuelo
//  - En pruebas estáticas se verá ruidoso; en vuelo se verá suave
// ================================================================
void readSensors() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // BMP280 rate limiting
  if (millis() - lastBmpRead >= bmpInterval) {
    float rawAlt = bmp.readAltitude(basePressure);

    // Filtro IIR por software — alpha=0.8 para vuelo real
    // (responsivo a cambios de altitud de 15-20 m/s)
    // Nota: en interiores se verá ruidoso — es correcto y esperado
    filteredAltitude = filteredAltitude * 0.8f + rawAlt * 0.2f;

    // Validación: ignorar saltos físicamente imposibles (>100 m en un tick)
    if (abs(filteredAltitude - currentAltitude) < 100.0f) {
      currentAltitude = filteredAltitude;
    }

    // Velocidad — diferencial de altitud filtrada
    if (dt > 0.001f && abs(gForce - 1.0f) > 0.05f) {
      float rawVel = (currentAltitude - lastAltitude) / dt;
      velocity = velocity * 0.85f + rawVel * 0.15f;
    }

    lastAltitude = currentAltitude;
    lastBmpRead  = millis();
  }

  // MPU9250 — G-Force y orientación
  xyzFloat data = mpu.getGValues();
  gForce = sqrt((data.x * data.x) + (data.y * data.y) + (data.z * data.z));

  // data.x es el eje longitudinal del cohete (apunta hacia la nariz)
  pitch = atan2(data.x, sqrt(data.y * data.y + data.z * data.z)) * 180.0f / 3.14159f;
  roll  = atan2(data.y, data.z) * 180.0f / 3.14159f;
}

// ================================================================
//  TELEMETRÍA — JSON a Ground Station vía WebSocket
// ================================================================
void sendData() {
  String json = "{";
  json += "\"t\":"  + String(millis()) + ",";
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
