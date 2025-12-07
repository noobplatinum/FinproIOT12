#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Bluetooth A2DP Speaker
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include "nvs_flash.h"

// Config
const char* ssid = "RC_CAR_AP";
const char* password = "12345678";
const char* mqtt_server = "192.168.4.2";
const int mqtt_port = 1883;
const char* topic_move = "rc/move";

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

void setup_wifi();
void setup_i2s();
void setup_bluetooth();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_reconnect();
void setMotorState(int a, int b, int c, int d);
void moveCar(const char* cmd);
void bt_data_cb(const uint8_t *data, uint32_t len);
void bt_connection_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println(F("\n=== ESP32 CTRL ==="));

  pinMode(PIN_KIRI_MAJU, OUTPUT);
  pinMode(PIN_KIRI_MUNDUR, OUTPUT);
  pinMode(PIN_KANAN_MAJU, OUTPUT);
  pinMode(PIN_KANAN_MUNDUR, OUTPUT);
  setMotorState(LOW, LOW, LOW, LOW);

  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);

  setup_i2s();
  setup_bluetooth();

  Serial.println(F("READY"));
}

void loop() {
  if (!mqttClient.connected()) mqtt_reconnect();
  mqttClient.loop();
  delay(10);
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
  // Direct compare without String object
  char cmd[16];
  int len = length < 15 ? length : 15;
  for (int i = 0; i < len; i++) cmd[i] = (char)payload[i];
  cmd[len] = '\0';
  
  Serial.print(F("CMD:"));
  Serial.println(cmd);
  
  moveCar(cmd);
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