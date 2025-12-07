#include <Arduino.h>
#include <WiFi.h>

// ===========================
// KONFIGURASI WIFI ACCESS POINT
// ===========================
const char* ap_ssid = "RC_CAR_AP";
const char* ap_password = "12345678";

// Konfigurasi IP Address untuk Access Point
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   ESP32 ROOT - WiFi Access Point");
  Serial.println("========================================\n");

  // Konfigurasi IP Address untuk AP
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP Config Failed!");
  }

  // Mulai Access Point
  bool apStarted = WiFi.softAP(ap_ssid, ap_password);
  
  if (apStarted) {
    Serial.println("Access Point Started Successfully!");
    Serial.println("------------------------------------------");
    Serial.print("SSID        : ");
    Serial.println(ap_ssid);
    Serial.print("Password    : ");
    Serial.println(ap_password);
    Serial.print("AP IP       : ");
    Serial.println(WiFi.softAPIP());
    Serial.println("------------------------------------------");
    Serial.println("\nConnect your devices to this WiFi network.");
    Serial.println("Waiting for connections...\n");
  } else {
    Serial.println("Failed to start Access Point!");
  }
}

void loop() {
  // Tampilkan jumlah device yang terconnect setiap 10 detik
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    
    int numClients = WiFi.softAPgetStationNum();
    Serial.print("[INFO] Connected devices: ");
    Serial.println(numClients);
  }
  
  delay(100);
}
