#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include "nvs_flash.h"

#define I2S_LRC     25  // LR Clock (Word Select)
#define I2S_BCLK    33  // Bit Clock
#define I2S_DIN     32  // Data In (to MAX98357)
#define DEVICE_NAME "ESP32_Speaker_Stream"

void setup_i2s() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100, // Standard Bluetooth audio rate
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

// This function is called when audio data is received via Bluetooth
void bt_data_cb(const uint8_t *data, uint32_t len) {
    size_t bytes_written;
    // Write the received audio data directly to the I2S peripheral
    i2s_write(I2S_NUM_0, data, len, &bytes_written, portMAX_DELAY);
}

// This function handles Bluetooth connection events
void bt_connection_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                Serial.println("Bluetooth Connected");
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                Serial.println("Bluetooth Disconnected");
            }
            break;
        case ESP_A2D_AUDIO_STATE_EVT:
            if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
                Serial.println("Audio Playing");
            } else {
                Serial.println("Audio Stopped");
            }
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32 Speaker...");

    // Initialize NVS (Required for Bluetooth)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    setup_i2s();

    if (!btStart()) {
        Serial.println("Failed to initialize Bluetooth controller");
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        Serial.println("Failed to initialize Bluedroid");
        return;
    }
    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.println("Failed to enable Bluedroid");
        return;
    }

    esp_bt_dev_set_device_name(DEVICE_NAME);

    // Initialize A2DP Sink
    // Register callback for connection status
    esp_a2d_register_callback(&bt_connection_cb);
    // Register callback for audio data stream
    esp_a2d_sink_register_data_callback(bt_data_cb);
    // Start A2DP Sink
    if (esp_a2d_sink_init() != ESP_OK) {
        Serial.println("Failed to initialize A2DP Sink");
        return;
    }

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    Serial.println("Bluetooth Speaker Ready.");
    Serial.print("Device Name: ");
    Serial.println(DEVICE_NAME);
}

void loop() {
    delay(1000);
}
