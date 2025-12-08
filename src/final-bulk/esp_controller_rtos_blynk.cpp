// Blynk config - MUST be before includes
#define BLYNK_TEMPLATE_ID "TMPL6E5eIpJrO"
#define BLYNK_TEMPLATE_NAME "Finpro IOT"
#define BLYNK_AUTH_TOKEN "CBoDkWk3RmGg_2zzPMqJVbU6ZhSf9EoQ"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>

// Bluetooth A2DP Speaker
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include "nvs_flash.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// WiFi Config
const char* ssid = "RC_CAR_AP";
const char* password = "12345678";

// MQTT Config (fallback)
const char* mqtt_server = "192.168.4.2";
const int mqtt_port = 1883;
const char* topic_move = "rc/move";

// Control mode flag
bool useBlynk = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Motor pins
#define PIN_KIRI_MUNDUR 26
#define PIN_KIRI_MAJU   27
#define PIN_KANAN_MAJU  14
#define PIN_KANAN_MUNDUR 12

// I2S pins
#define I2S_LRC  25
#define I2S_BCLK 33
#define I2S_DIN  32
#define BT_NAME "RC_CAR_Speaker"

// ====== RTOS Handles ======
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t blynkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
QueueHandle_t motorCommandQueue;

// Motor command structure
typedef struct {
  char cmd[16];
} MotorCommand_t;

// Function declarations
void setup_wifi();
void setup_i2s();
void setup_bluetooth();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_reconnect();
void setMotorState(int a, int b, int c, int d);
void moveCar(const char* cmd);
void bt_data_cb(const uint8_t *data, uint32_t len);
void bt_connection_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
bool tryBlynkConnection();

// Helper function to send motor command to queue
void sendMotorCommand(const char* cmd) {
  MotorCommand_t motorCmd;
  strncpy(motorCmd.cmd, cmd, 15);
  motorCmd.cmd[15] = '\0';
  
  Serial.print(F("CMD:"));
  Serial.println(cmd);
  
  if (xQueueSend(motorCommandQueue, &motorCmd, 0) != pdTRUE) {
    Serial.println(F("[RTOS] Queue full!"));
  }
}

// ====== Blynk Virtual Pin Handlers ======
// V0 = UP, V1 = DOWN, V2 = LEFT, V3 = RIGHT, V4 = STOP
BLYNK_WRITE(V0) {
  if (param.asInt() == 1) {
    sendMotorCommand("UP");
  } else {
    sendMotorCommand("STOP");
  }
}

BLYNK_WRITE(V1) {
  if (param.asInt() == 1) {
    sendMotorCommand("DOWN");
  } else {
    sendMotorCommand("STOP");
  }
}

BLYNK_WRITE(V2) {
  if (param.asInt() == 1) {
    sendMotorCommand("LEFT");
  } else {
    sendMotorCommand("STOP");
  }
}

BLYNK_WRITE(V3) {
  if (param.asInt() == 1) {
    sendMotorCommand("RIGHT");
  } else {
    sendMotorCommand("STOP");
  }
}

BLYNK_WRITE(V4) {
  sendMotorCommand("STOP");
}

// Blynk connected callback
BLYNK_CONNECTED() {
  Serial.println(F("[Blynk] Connected to cloud!"));
}

BLYNK_DISCONNECTED() {
  Serial.println(F("[Blynk] Disconnected from cloud!"));
}

// ====== RTOS Tasks ======
void blynkTask(void *parameter) {
  Serial.println(F("[RTOS] Blynk Task started on Core 1"));
  
  for(;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[Blynk] WiFi lost, reconnecting..."));
      setup_wifi();
    }
    Blynk.run();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void mqttTask(void *parameter) {
  Serial.println(F("[RTOS] MQTT Task started on Core 1"));
  
  for(;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[MQTT] WiFi lost, reconnecting..."));
      setup_wifi();
    }
    
    if (!mqttClient.connected()) {
      mqtt_reconnect();
    }
    mqttClient.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void motorTask(void *parameter) {
  Serial.println(F("[RTOS] Motor Task started on Core 0"));
  MotorCommand_t receivedCmd;
  
  for(;;) {
    // Wait for command from queue (blocks until command available)
    if (xQueueReceive(motorCommandQueue, &receivedCmd, portMAX_DELAY) == pdTRUE) {
      moveCar(receivedCmd.cmd);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println(F("\n=== ESP32 CTRL (RTOS) ==="));

  // Initialize motor pins
  pinMode(PIN_KIRI_MAJU, OUTPUT);
  pinMode(PIN_KIRI_MUNDUR, OUTPUT);
  pinMode(PIN_KANAN_MAJU, OUTPUT);
  pinMode(PIN_KANAN_MUNDUR, OUTPUT);
  setMotorState(LOW, LOW, LOW, LOW);

  // Initialize WiFi
  setup_wifi();

  // Create command queue (holds up to 10 commands)
  motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand_t));
  if (motorCommandQueue == NULL) {
    Serial.println(F("[RTOS] Failed to create queue!"));
  }

  // ====== Try Blynk first, then fallback to MQTT ======
  Serial.println(F("[Control] Trying Blynk Cloud..."));
  
  // Configure Blynk (non-blocking mode)
  Blynk.config(BLYNK_AUTH_TOKEN);
  
  // Try to connect to Blynk with 10 second timeout
  unsigned long startAttempt = millis();
  while (!Blynk.connect() && (millis() - startAttempt < 10000)) {
    delay(500);
    Serial.print(".");
  }
  
  if (Blynk.connected()) {
    useBlynk = true;
    Serial.println(F("\n[Control] Using BLYNK mode"));
    
    // Create Blynk Task on Core 1
    xTaskCreatePinnedToCore(
      blynkTask,          // Task function
      "BlynkTask",        // Task name
      4096,               // Stack size
      NULL,               // Parameters
      2,                  // Priority
      &blynkTaskHandle,   // Task handle
      1                   // Core 1
    );
  } else {
    useBlynk = false;
    Serial.println(F("\n[Control] Blynk failed! Using MQTT fallback"));
    
    // Setup MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqtt_callback);
    
    // Create MQTT Task on Core 1
    xTaskCreatePinnedToCore(
      mqttTask,           // Task function
      "MQTTTask",         // Task name
      4096,               // Stack size
      NULL,               // Parameters
      2,                  // Priority
      &mqttTaskHandle,    // Task handle
      1                   // Core 1
    );
  }

  // Initialize Bluetooth Audio
  setup_i2s();
  setup_bluetooth();

  // Create Motor Task on Core 0 (same as Bluetooth for low latency)
  xTaskCreatePinnedToCore(
    motorTask,          // Task function
    "MotorTask",        // Task name
    2048,               // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &motorTaskHandle,   // Task handle
    0                   // Core 0
  );

  Serial.println(F("[RTOS] All tasks created successfully"));
  Serial.print(F("Mode: "));
  Serial.println(useBlynk ? F("BLYNK") : F("MQTT"));
  Serial.println(F("READY"));
}

void loop() {
  // Empty - FreeRTOS tasks handle everything
  vTaskDelay(portMAX_DELAY);
}

void setup_wifi() {
  Serial.print(F("WiFi.."));
  WiFi.begin(ssid, password);
  
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 30) {
    delay(500);
    Serial.print('.');
    t++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("OK IP:"));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("FAIL"));
  }
}

void setup_i2s() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };
  
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
}

void setup_bluetooth() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  if (!btStart()) return;
  if (esp_bluedroid_init() != ESP_OK) return;
  if (esp_bluedroid_enable() != ESP_OK) return;

  esp_bt_dev_set_device_name(BT_NAME);
  esp_a2d_register_callback(&bt_connection_cb);
  esp_a2d_sink_register_data_callback(bt_data_cb);
  
  if (esp_a2d_sink_init() != ESP_OK) return;
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  
  Serial.print(F("BT:"));
  Serial.println(BT_NAME);
}

void bt_data_cb(const uint8_t *data, uint32_t len) {
  size_t w;
  i2s_write(I2S_NUM_0, data, len, &w, portMAX_DELAY);
}

void bt_connection_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
      Serial.println(F("BT+"));
    } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
      Serial.println(F("BT-"));
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Create command structure and copy payload
  MotorCommand_t cmd;
  int len = length < 15 ? length : 15;
  for (int i = 0; i < len; i++) cmd.cmd[i] = (char)payload[i];
  cmd.cmd[len] = '\0';
  
  Serial.print(F("CMD:"));
  Serial.println(cmd.cmd);
  
  // Send command to motor queue (non-blocking)
  if (xQueueSend(motorCommandQueue, &cmd, 0) != pdTRUE) {
    Serial.println(F("[RTOS] Queue full, command dropped!"));
  }
}
/*
C:\Windows\System32>net stop mosquitto
The Mosquitto Broker service is stopping.
The Mosquitto Broker service was stopped successfully.


C:\Windows\System32>mosquitto -c "c:\Users\daffa\OneDrive\Documents\PlatformIO\Projects\IOT-Finpro\server\mosquitto.conf" -v
*/
void mqtt_reconnect() {
  if (!mqttClient.connected()) {
    char cid[20];
    snprintf(cid, 20, "ESP%X", random(0xffff));
    
    if (mqttClient.connect(cid)) {
      Serial.println(F("MQTT+"));
      mqttClient.subscribe(topic_move);
    } else {
      delay(2000);
    }
  }
}

void setMotorState(int a, int b, int c, int d) {
  digitalWrite(PIN_KIRI_MAJU, a);
  digitalWrite(PIN_KIRI_MUNDUR, b);
  digitalWrite(PIN_KANAN_MAJU, c);
  digitalWrite(PIN_KANAN_MUNDUR, d);
}

void moveCar(const char* cmd) {
  if (strcmp(cmd, "UP") == 0 || strcmp(cmd, "W") == 0) {
    setMotorState(HIGH, LOW, HIGH, LOW);
    Serial.println(F("^MAJU"));
  } 
  else if (strcmp(cmd, "DOWN") == 0 || strcmp(cmd, "S") == 0) {
    setMotorState(LOW, HIGH, LOW, HIGH);
    Serial.println(F("vMUNDUR"));
  } 
  else if (strcmp(cmd, "LEFT") == 0 || strcmp(cmd, "A") == 0) {
    setMotorState(LOW, HIGH, HIGH, LOW);
    Serial.println(F("<KIRI"));
  } 
  else if (strcmp(cmd, "RIGHT") == 0 || strcmp(cmd, "D") == 0) {
    setMotorState(HIGH, LOW, LOW, HIGH);
    Serial.println(F("KANAN>"));
  } 
  else if (strcmp(cmd, "STOP") == 0) {
    setMotorState(LOW, LOW, LOW, LOW);
    Serial.println(F("=STOP"));
  }
}
