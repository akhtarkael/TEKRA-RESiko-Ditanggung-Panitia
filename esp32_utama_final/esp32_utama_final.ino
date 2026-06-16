#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============================================
// Konfig Wifi dan MQTT
// =============================================
const char* SSID          = "your-ssid"; //sesuaikan nama wifi yang dipakai
const char* WIFI_PASSWORD = "your-passwd"; //sesuaikan password wifi yang dipakai
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;

const char* TOPIC_SENSOR    = "tekra2026/RESikoDitanggungPanitia/esp32/sensor";
const char* TOPIC_CMD_LAMPU = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/lampu";
const char* TOPIC_CMD_MOTOR = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/motor";
const char* TOPIC_CMD_SERVO = "tekra2026/RESikoDitanggungPanitia/esp32/cmd/servo";

// =============================================
// Pinout
// =============================================
#define DHTPIN     32
#define DHTTYPE    DHT11
#define PZEM_RX    16
#define PZEM_TX    4
#define OLED_SDA   23
#define OLED_SCL   22
#define OLED_W     128
#define OLED_H     64

const int pir1      = 34;
const int pir2      = 18;
const int ledPin    = 21;
const int gasPin    = 33;
const int ldrPin    = 35;
const int relayPin  = 27;
const int motorPin  = 25;
const int buttonPin = 15;
const int saklarPin = 17;

// =============================================
// Objek
// =============================================
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
PZEM004Tv30* pzem = nullptr;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

// =============================================
// Struct dari data
// =============================================
struct DataSensor {
  int kelembapan;
  int suhu;
  int gas;
  int kecerahan;
  int gerakan;
  int statusMotor;
  int statusLampu;
  int saklar;
  float tegangan;
  float arus;
  float daya;
  float energi;
  float frekuensi;
  float powerFactor;
};

DataSensor dataSistemShared = {0, 0, 0, 0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

volatile int   cmdLampu = 0;
volatile int   cmdMotor = 0;
volatile int   cmdRelay = 0;

volatile bool motorToggleState   = false;
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE dataMux  = portMUX_INITIALIZER_UNLOCKED;

volatile bool _mqttOk = false;
SemaphoreHandle_t _mqttMutex = NULL;

// =============================================
// Deklarasi Task buat RTOS
// =============================================
void TaskBacaSensor(void *pvParameters);
void TaskEksekusiAktuator(void *pvParameters);
void TaskPublishSensor(void *pvParameters);
void TaskMQTTLoop(void *pvParameters);
void TaskSerialLog(void *pvParameters);
void TaskOLED(void *pvParameters);

// =============================================
// ISR button
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
// MQTT Callback
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
    cmdRelay = doc["status"];
    portEXIT_CRITICAL(&dataMux);
  }
}

// =============================================
// Setup
// =============================================
void setup() {
  Serial.begin(115200);
  pzem = new PZEM004Tv30(Serial2, PZEM_RX, PZEM_TX);
  dht.begin();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] Gagal init! Cek wiring.");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 28);
    display.print("Inisialisasi...");
    display.display();
  }

  pinMode(pir1,      INPUT);
  pinMode(pir2,      INPUT_PULLDOWN);
  
  pinMode(ledPin,    OUTPUT);
  pinMode(gasPin,    INPUT);
  pinMode(ldrPin,    INPUT);
  pinMode(motorPin,  OUTPUT);
  pinMode(relayPin,  OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(saklarPin, INPUT_PULLUP);

  digitalWrite(relayPin, HIGH);

  Serial.print("Konek WiFi...");
  WiFi.begin(SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" OK!");

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  esp_task_wdt_deinit();
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms     = 15000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&twdt_config);

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);

  _mqttMutex = xSemaphoreCreateMutex();

  xTaskCreate(TaskBacaSensor,       "BacaSensor", 8192, NULL, 2, NULL);
  xTaskCreate(TaskEksekusiAktuator, "Aktuator",   2048, NULL, 2, NULL);
  xTaskCreate(TaskPublishSensor,    "Publish",    6144, NULL, 1, NULL);
  xTaskCreate(TaskMQTTLoop,         "MQTTLoop",   8192, NULL, 1, NULL);
  xTaskCreate(TaskSerialLog,        "SerialLog",  4096, NULL, 1, NULL);
  xTaskCreate(TaskOLED,             "OLED",       4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// =============================================
// Task: Baca Sensor dan Saklar Global
// =============================================
void TaskBacaSensor(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    int gas   = analogRead(gasPin);
    int ldr   = analogRead(ldrPin);
    int pir   = digitalRead(pir1) || digitalRead(pir2);
    float hum  = dht.readHumidity();
    float temp = dht.readTemperature();

    int saklarStatus = (digitalRead(saklarPin) == LOW) ? 1 : 0;

    float pzemV  = pzem->voltage();
    float pzemA  = pzem->current();
    float pzemW  = pzem->power();
    float pzemKwh = pzem->energy();
    float pzemHz = pzem->frequency();
    float pzemPF = pzem->pf();

    bool pzemOk = !isnan(pzemV) && !isinf(pzemV) && pzemV > 180.0f && pzemV < 265.0f;
    if (!pzemOk) Serial.println("[PZEM] WARN: tidak terbaca / tidak ada AC");

    portENTER_CRITICAL(&dataMux);
    dataSistemShared.gas       = gas;
    dataSistemShared.kecerahan = ldr;
    dataSistemShared.gerakan   = pir;
    dataSistemShared.saklar    = saklarStatus;
    if (!isnan(hum)  && hum  > 0)    dataSistemShared.kelembapan = (int)hum;
    if (!isnan(temp) && temp > -40.0f && temp < 80.0f) dataSistemShared.suhu = (int)temp;
    if (pzemOk) {
      dataSistemShared.tegangan = pzemV;
    } else {
      dataSistemShared.tegangan    = 0.0f;
      dataSistemShared.arus        = 0.0f;
      dataSistemShared.daya        = 0.0f;
    }
    if (!isnan(pzemA)   && !isinf(pzemA)   && pzemA   >= 0.0f   && pzemA   <= 100.0f)   dataSistemShared.arus        = pzemA;
    if (!isnan(pzemW)   && !isinf(pzemW)   && pzemW   >= 0.0f   && pzemW   <= 23000.0f) dataSistemShared.daya        = pzemW;
    if (!isnan(pzemKwh) && !isinf(pzemKwh) && pzemKwh >= 0.0f   && pzemKwh <= 9999.999f) dataSistemShared.energi    = pzemKwh;
    if (!isnan(pzemHz)  && !isinf(pzemHz)  && pzemHz  > 40.0f   && pzemHz  < 70.0f)     dataSistemShared.frekuensi  = pzemHz;
    if (!isnan(pzemPF)  && !isinf(pzemPF)  && pzemPF  >= 0.0f   && pzemPF  <= 1.0f)     dataSistemShared.powerFactor = pzemPF;
    portEXIT_CRITICAL(&dataMux);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// =============================================
// Task: Aktuator
// =============================================
void TaskEksekusiAktuator(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    int localLampu, localMotor, localRelay;
    portENTER_CRITICAL(&dataMux);
    localLampu = cmdLampu;
    localMotor = cmdMotor;
    localRelay = cmdRelay;
    portEXIT_CRITICAL(&dataMux);
    digitalWrite(ledPin,   localLampu ? HIGH : LOW);
    digitalWrite(motorPin, localMotor ? HIGH : LOW);
    digitalWrite(relayPin, localRelay ? LOW : HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// =============================================
// Task: Publish Sensor
// =============================================
void TaskPublishSensor(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    if (_mqttOk) {
      DataSensor d;
      bool toggleCopy;
      portENTER_CRITICAL(&dataMux);
      d = dataSistemShared;
      portEXIT_CRITICAL(&dataMux);
      portENTER_CRITICAL(&timerMux);
      toggleCopy = motorToggleState;
      portEXIT_CRITICAL(&timerMux);

      StaticJsonDocument<400> doc;
      doc["kelembapan"]   = d.kelembapan;
      doc["suhu"]         = d.suhu;
      doc["gas"]          = d.gas;
      doc["kecerahan"]    = d.kecerahan;
      doc["gerakan"]      = d.gerakan;
      doc["toggleMotor"]  = toggleCopy ? 1 : 0;
      doc["saklar"]       = d.saklar;
      doc["tegangan"]     = d.tegangan;
      doc["arus"]         = d.arus;
      doc["daya"]         = d.daya;
      doc["energi"]       = d.energi;
      doc["frekuensi"]    = d.frekuensi;
      doc["powerFactor"]  = d.powerFactor;

      char buffer[384];
      serializeJson(doc, buffer);
      if (xSemaphoreTake(_mqttMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        mqttClient.publish(TOPIC_SENSOR, buffer);
        xSemaphoreGive(_mqttMutex);
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// =============================================
// Task: Loop MQTT dan Reconnecting
// =============================================
void TaskMQTTLoop(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();

    bool isConn = false;
    if (xSemaphoreTake(_mqttMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      isConn = mqttClient.connected();
      xSemaphoreGive(_mqttMutex);
    }

    if (!isConn) {
      _mqttOk = false;
      Serial.print("Reconnect HiveMQ...");
      String clientId = "ESP32Utama-" + String(random(0xffff), HEX);
      bool ok = false;
      if (xSemaphoreTake(_mqttMutex, pdMS_TO_TICKS(8000)) == pdTRUE) {
        ok = mqttClient.connect(clientId.c_str());
        if (ok) {
          mqttClient.subscribe(TOPIC_CMD_LAMPU);
          mqttClient.subscribe(TOPIC_CMD_MOTOR);
          mqttClient.subscribe(TOPIC_CMD_SERVO);
        }
        xSemaphoreGive(_mqttMutex);
      }
      if (ok) {
        _mqttOk = true;
        Serial.println(" OK!");
      } else {
        Serial.println(" Gagal, coba lagi 3 detik...");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
      }
    } else {
      if (xSemaphoreTake(_mqttMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        mqttClient.loop();
        xSemaphoreGive(_mqttMutex);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =============================================
// Task: Serial Log
// =============================================
void TaskSerialLog(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
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
    Serial.println(_mqttOk ? "TERKONEKSI" : "TERPUTUS");
    Serial.print("  [LAYER 3] Saklar Global: "); Serial.println(d.saklar ? "ON (Sistem Normal)" : "OFF (Mode Hemat)");
    Serial.print("  [SENSOR] Suhu        : "); Serial.print(d.suhu); Serial.println(" °C");
    Serial.print("  [SENSOR] Kelembapan  : "); Serial.print(d.kelembapan); Serial.println(" %");
    Serial.print("  [SENSOR] Gas         : "); Serial.println(d.gas);
    Serial.print("  [SENSOR] Kecerahan   : "); Serial.println(d.kecerahan);
    Serial.print("  [SENSOR] Gerakan     : "); Serial.println(d.gerakan ? "TERDETEKSI (!)" : "Tidak Terdeteksi");
    Serial.print("  [PZEM]   Tegangan    : "); Serial.print(d.tegangan, 1); Serial.println(" V");
    Serial.print("  [PZEM]   Arus        : "); Serial.print(d.arus, 3); Serial.println(" A");
    Serial.print("  [PZEM]   Daya        : "); Serial.print(d.daya, 1); Serial.println(" W");
    Serial.print("  [PZEM]   Energi      : "); Serial.print(d.energi, 3); Serial.println(" kWh");
    Serial.print("  [PZEM]   Frekuensi   : "); Serial.print(d.frekuensi, 1); Serial.println(" Hz");
    Serial.print("  [PZEM]   Power Factor: "); Serial.println(d.powerFactor, 2);
    Serial.print("  [CMD]    Lampu       : "); Serial.println(cmdLampu ? "NYALA" : "MATI");
    Serial.print("  [CMD]    Motor       : "); Serial.println(cmdMotor ? "NYALA" : "MATI");
    Serial.print("  [CMD]    Main Relay  : "); Serial.println(cmdRelay ? "ON" : "OFF");
    Serial.print("  [BTN]    Toggle Motor: "); Serial.println(toggleCopy ? "ON" : "OFF");
    Serial.println("=====================================");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// =============================================
// Task: OLED Display
// =============================================
void TaskOLED(void *pvParameters) {
  (void) pvParameters;
  esp_task_wdt_add(NULL);

  // Splash screen HYDRA
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(22, 18);
  display.print("HYDRA");
  display.setTextSize(1);
  display.setCursor(22, 44);
  display.print("Sistem Monitoring");
  display.display();
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  for (;;) {
    esp_task_wdt_reset();
    DataSensor d;
    portENTER_CRITICAL(&dataMux);
    d = dataSistemShared;
    portEXIT_CRITICAL(&dataMux);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(14, 1);
    display.print("[ DAYA LISTRIK ]");
    display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 15);
    display.print("V:");
    display.print(d.tegangan, 1);

    display.setCursor(0, 33);
    display.print("A:");
    display.print(d.arus, 3);

    display.setCursor(0, 49);
    display.print(d.energi, 3);
    display.print("kWh");

    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}