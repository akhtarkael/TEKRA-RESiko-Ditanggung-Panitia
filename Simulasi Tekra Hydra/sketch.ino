#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

// =============================================
// KONFIGURASI WiFi & MQTT
// =============================================
const char* SSID          = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;

const char* TOPIC_SENSOR    = "tekra2026/RESikoDitanggungPanitia/esp32/sensor";
const char* TOPIC_CMD_LAMPU = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/lampu";
const char* TOPIC_CMD_MOTOR = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/motor";
const char* TOPIC_CMD_SERVO = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/servo";

// =============================================
// PIN DEFINITION
// =============================================
#define DHTPIN     32
#define DHTTYPE    DHT22

const int pir1      = 34;
const int pir2      = 18;
const int ledPin    = 21;
const int gasPin    = 33;
const int ldrPin    = 35;
const int servoPin  = 27;
const int motorPin  = 25;
const int buttonPin = 15;
const int saklarPin = 17;

// =============================================
// OBJEK
// =============================================
DHT dht(DHTPIN, DHTTYPE);
Servo myservo;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// =============================================
// SHARED DATA
// =============================================
struct DataSensor {
  int kelembapan;
  int gas;
  int kecerahan;
  int gerakan;
  int statusMotor;
  int statusLampu;
  int saklar;
};

DataSensor dataSistemShared = {0, 0, 0, 0, 0, 0, 1};

volatile int  cmdLampu = 0;
volatile int  cmdMotor = 0;
volatile int  cmdServo = 0;

volatile bool      motorToggleState = false;
volatile unsigned long lastDebounceTime = 0;
const unsigned long    debounceDelay    = 500;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE dataMux  = portMUX_INITIALIZER_UNLOCKED;

// =============================================
// TASK DECLARATIONS
// =============================================
void TaskBacaSensor      (void *pvParameters);
void TaskEksekusiAktuator(void *pvParameters);
void TaskPublishSensor   (void *pvParameters);
void TaskMQTTLoop        (void *pvParameters);
void TaskSerialLog       (void *pvParameters);

// =============================================
// BUTTON ISR
// =============================================
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (digitalRead(buttonPin) == LOW) {
      portENTER_CRITICAL_ISR(&timerMux);
      motorToggleState = !motorToggleState;
      portEXIT_CRITICAL_ISR(&timerMux);
      lastDebounceTime = currentTime;
    }
  }
}

// =============================================
// MQTT CALLBACK
// =============================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  StaticJsonDocument<64> doc;
  deserializeJson(doc, payload, length);

  if (topicStr == TOPIC_CMD_LAMPU) {
    portENTER_CRITICAL(&dataMux);
    cmdLampu = doc["status"];
    portEXIT_CRITICAL(&dataMux);
  }
  else if (topicStr == TOPIC_CMD_MOTOR) {
    portENTER_CRITICAL(&dataMux);
    cmdMotor = doc["status"];
    portEXIT_CRITICAL(&dataMux);
  }
  else if (topicStr == TOPIC_CMD_SERVO) {
    portENTER_CRITICAL(&dataMux);
    cmdServo = doc["angle"];
    portEXIT_CRITICAL(&dataMux);
  }
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("Memulai ESP32...");

  dht.begin();

  pinMode(pir1,      INPUT);
  pinMode(pir2,      INPUT_PULLDOWN);
  pinMode(ledPin,    OUTPUT);
  pinMode(gasPin,    INPUT);
  pinMode(ldrPin,    INPUT);
  pinMode(motorPin,  OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(saklarPin, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);
  myservo.write(0);

  Serial.print("Konek WiFi Wokwi...");
  WiFi.begin(SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
  }
  Serial.println(" OK!");

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  // Menonaktifkan total TWDT agar simulasi tidak sensitif terhadap lag browser/Wokwi
  esp_task_wdt_deinit();

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);

  // Menggunakan xTaskCreatePinnedToCore untuk stabilitas interupsi di Wokwi
  xTaskCreatePinnedToCore(TaskBacaSensor,       "BacaSensor",  3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskEksekusiAktuator, "Aktuator",    2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskPublishSensor,    "Publish",     4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskMQTTLoop,         "MQTTLoop",    4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskSerialLog,        "SerialLog",   2048, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// =============================================
// TASK: BACA SENSOR + SAKLAR GLOBAL
// =============================================
void TaskBacaSensor(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    int gas  = analogRead(gasPin);
    int ldr  = analogRead(ldrPin);
    int pir  = (digitalRead(pir1) == HIGH) || (digitalRead(pir2) == HIGH);
    int saklarStatus = (digitalRead(saklarPin) == LOW) ? 1 : 0;
    
    // Membaca DHT diberi jeda yield internal agar tidak mengunci interrupt core
    float hum = dht.readHumidity(); 
    if (isnan(hum) || hum < 0) {
      hum = 0;
    }

    portENTER_CRITICAL(&dataMux);
    dataSistemShared.gas       = gas;
    dataSistemShared.kecerahan = ldr;
    dataSistemShared.gerakan   = pir;
    dataSistemShared.saklar    = saklarStatus;
    dataSistemShared.kelembapan = (int)hum;
    portEXIT_CRITICAL(&dataMux);

    vTaskDelay(300 / portTICK_PERIOD_MS); // Naikkan jeda baca ke 300ms agar CPU stabil
  }
}

// =============================================
// TASK: EKSEKUSI AKTUATOR
// =============================================
void TaskEksekusiAktuator(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    int localLampu, localMotor, localServo;
    portENTER_CRITICAL(&dataMux);
    localLampu = cmdLampu;
    localMotor = cmdMotor;
    localServo = cmdServo;
    portEXIT_CRITICAL(&dataMux);
    
    digitalWrite(ledPin,   localLampu ? HIGH : LOW);
    digitalWrite(motorPin, localMotor ? HIGH : LOW);
    myservo.write(localServo);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// =============================================
// TASK: PUBLISH SENSOR + TOGGLE STATE + SAKLAR
// =============================================
void TaskPublishSensor(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (mqttClient.connected()) {
      DataSensor d;
      bool toggleCopy;
      portENTER_CRITICAL(&dataMux);
      d = dataSistemShared;
      portEXIT_CRITICAL(&dataMux);
      portENTER_CRITICAL(&timerMux);
      toggleCopy = motorToggleState;
      portEXIT_CRITICAL(&timerMux);

      StaticJsonDocument<192> doc;
      doc["kelembapan"]  = d.kelembapan;
      doc["gas"]         = d.gas;
      doc["kecerahan"]   = d.kecerahan;
      doc["gerakan"]     = d.gerakan;
      doc["toggleMotor"] = toggleCopy ? 1 : 0;
      doc["saklar"]      = d.saklar;

      char buffer[192];
      serializeJson(doc, buffer);
      mqttClient.publish(TOPIC_SENSOR, buffer);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Publish 2 detik sekali agar jaringan virtual Wokwi aman
  }
}

// =============================================
// TASK: MQTT LOOP + RECONNECT
// =============================================
void TaskMQTTLoop(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (!mqttClient.connected()) {
      Serial.print("Reconnect HiveMQ...");
      String clientId = "ESP32Utama-" + String(random(0xffff), HEX);
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println(" OK!");
        mqttClient.subscribe(TOPIC_CMD_LAMPU);
        mqttClient.subscribe(TOPIC_CMD_MOTOR);
        mqttClient.subscribe(TOPIC_CMD_SERVO);
      } else {
        Serial.println(" Gagal, coba lagi 3 detik...");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
      }
    }
    mqttClient.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS); // Diturunkan ke 100ms untuk mencegah penumpukan background network loop
  }
}

// =============================================
// TASK: SERIAL LOG
// =============================================
void TaskSerialLog(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    DataSensor d;
    bool toggleCopy;
    portENTER_CRITICAL(&dataMux);
    d = dataSistemShared;
    portEXIT_CRITICAL(&dataMux);
    portENTER_CRITICAL(&timerMux);
    toggleCopy = motorToggleState;
    portEXIT_CRITICAL(&timerMux);

    Serial.println("\n=======[ ESP32 UTAMA STATUS ]=======");
    Serial.print("  [MQTT]   Status      : ");
    Serial.println(mqttClient.connected() ? "TERKONEKSI" : "TERPUTUS");
    vTaskDelay(10 / portTICK_PERIOD_MS); 

    Serial.print("  [LAYER 3] Saklar Global: "); Serial.println(d.saklar ? "ON (Sistem Normal)" : "OFF (Mode Hemat)");
    Serial.print("  [SENSOR] Kelembapan  : "); Serial.print(d.kelembapan); Serial.println(" %");
    Serial.print("  [SENSOR] Gas         : "); Serial.println(d.gas);
    vTaskDelay(10 / portTICK_PERIOD_MS); 

    Serial.print("  [SENSOR] Kecerahan   : "); Serial.println(d.kecerahan);
    Serial.print("  [SENSOR] Gerakan     : "); Serial.println(d.gerakan ? "TERDETEKSI (!)" : "Tidak Terdeteksi");
    Serial.print("  [CMD]    Lampu       : "); Serial.println(cmdLampu ? "NYALA" : "MATI");
    vTaskDelay(10 / portTICK_PERIOD_MS); 

    Serial.print("  [CMD]    Motor       : "); Serial.println(cmdMotor ? "NYALA" : "MATI");
    Serial.print("  [CMD]    Servo       : "); Serial.print(cmdServo); Serial.println("°");
    Serial.print("  [BTN]    Toggle Motor: "); Serial.println(toggleCopy ? "ON" : "OFF");
    Serial.println("=====================================");

    vTaskDelay(4000 / portTICK_PERIOD_MS); // Log muncul setiap 4 detik demi kestabilan runtime simulator
  }
}