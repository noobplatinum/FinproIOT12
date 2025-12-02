#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>

// ===========================
// KONFIGURASI WIFI & MQTT
// ===========================
const char* ssid = "[VR]";
const char* password = "Taitai12";
const char* mqtt_server = "broker.hivemq.com";
// Topik MQTT (gunakan yang unik)
const char* topic_move = "myrc/antonproject/move"; 

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(80);

// ===========================
// DEFINISI 4 PIN KONTROL (Original ESP32 ke L298N)
// ===========================
#define PIN_KIRI_MUNDUR 26  // GPIO 26
#define PIN_KIRI_MAJU   27  // GPIO 27
#define PIN_KANAN_MAJU  14  // GPIO 14
#define PIN_KANAN_MUNDUR 12 // GPIO 12

// --- Halaman Web HTML + JS untuk Kontrol WASD ---
const char htmlHomePage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>RC Car mDNS Hub</title>
  <style>
    body { font-family: Arial; text-align: center; margin:0; background-color: #222; color: white; overflow: hidden;}
    #videoContainer { width: 100%; height: 100vh; display: flex; justify-content: center; align-items: center; }
    #videoStream { max-width: 100%; max-height: 100%; object-fit: contain; }
    .overlay-info { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.5); padding: 10px; border-radius: 5px; text-align: left;}
    .controls-hint { position: absolute; bottom: 20px; width: 100%; text-align: center; font-weight: bold; text-shadow: 1px 1px 2px black;}
  </style>
</head>
<body>
  <div id="videoContainer">
    <img id="videoStream" src="http://camcar.local/stream" alt="Menunggu koneksi kamera (camcar.local)...">
  </div>
  <div class="overlay-info">
    <b>Status MQTT:</b> <span id="mqttStatus" style="color:red">Connecting...</span><br>
    <b>Last Cmd:</b> <span id="lastCmd">-</span>
  </div>
  <div class="controls-hint">Gunakan Keyboard: W A S D atau Panah Arah</div>

  <script src="https://cdnjs.cloudflare.com/ajax/libs/mqtt/4.3.7/mqtt.min.js"></script>
  <script>
    const statusSpan = document.getElementById('mqttStatus');
    const cmdSpan = document.getElementById('lastCmd');
    let activeKey = null; 

    // Koneksi ke Broker Public HiveMQ via WebSocket
    const client = mqtt.connect('ws://broker.hivemq.com:8000/mqtt');

    client.on('connect', function () {
      statusSpan.innerHTML = "Connected";
      statusSpan.style.color = "lime";
      console.log('Browser connected to MQTT Broker');
    });

    client.on('error', function (err) {
      statusSpan.innerHTML = "Error";
      statusSpan.style.color = "red";
      console.error('MQTT Error:', err);
    });

    function pub(command) {
      if(client.connected) {
          client.publish('myrc/antonproject/move', command);
          cmdSpan.innerHTML = command;
      }
    }

    // Logika Keyboard WASD
    document.addEventListener('keydown', (e) => {
        if (activeKey) return; 
        const key = e.key.toLowerCase();
        if (key === 'w' || key === 'arrowup') { pub('UP'); activeKey = key; } 
        else if (key === 's' || key === 'arrowdown') { pub('DOWN'); activeKey = key; } 
        else if (key === 'a' || key === 'arrowleft') { pub('LEFT'); activeKey = key; } 
        else if (key === 'd' || key === 'arrowright') { pub('RIGHT'); activeKey = key; }
    });

    document.addEventListener('keyup', (e) => {
        const key = e.key.toLowerCase();
        if (key === activeKey) { pub('STOP'); activeKey = null; }
    });
  </script>
</body>
</html>
)=====";

// --- Deklarasi Fungsi ---
void setup_wifi();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_reconnect();
void setMotorState(int kiriMaju, int kiriMundur, int kananMaju, int kananMundur);
void moveCar(String command);
void handleRoot();

void setup() {
  Serial.begin(115200);

  // Setup 4 Pin Kontrol sebagai OUTPUT
  pinMode(PIN_KIRI_MAJU, OUTPUT);
  pinMode(PIN_KIRI_MUNDUR, OUTPUT);
  pinMode(PIN_KANAN_MAJU, OUTPUT);
  pinMode(PIN_KANAN_MUNDUR, OUTPUT);

  setMotorState(LOW, LOW, LOW, LOW);

  setup_wifi();

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);

  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server hub started");
}

void loop() {
  server.handleClient(); 
  if (!mqttClient.connected()) { mqtt_reconnect(); } 
  mqttClient.loop();
}

// --- Implementasi Fungsi ---

void handleRoot() {
  server.send(200, "text/html", htmlHomePage);
}

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");
  Serial.println("--------------------------------------------------");
  Serial.print("HUB READY. BUKA IP INI DI BROWSER LAPTOP: http://");
  Serial.println(WiFi.localIP());
  Serial.println("--------------------------------------------------");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String messageStr = "";
  for (int i = 0; i < length; i++) { messageStr += (char)payload[i]; }
  String topicStr = String(topic);

  if (topicStr == topic_move) {
    moveCar(messageStr);
  }
}

void mqtt_reconnect() {
  if (!mqttClient.connected()) {
    String clientId = "ESP32Hub-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT Connected");
      mqttClient.subscribe(topic_move);
    } else {
      delay(2000);
    }
  }
}

// --- FUNGSI KONTROL MOTOR (LOGIKA 4 PIN) ---
void setMotorState(int kiriMaju, int kiriMundur, int kananMaju, int kananMundur) {
  digitalWrite(PIN_KIRI_MAJU, kiriMaju);
  digitalWrite(PIN_KIRI_MUNDUR, kiriMundur);
  digitalWrite(PIN_KANAN_MAJU, kananMaju);
  digitalWrite(PIN_KANAN_MUNDUR, kananMundur);
}

void moveCar(String command) {
  if (command == "UP") {
    setMotorState(HIGH, LOW, HIGH, LOW);
  } else if (command == "DOWN") {
    setMotorState(LOW, HIGH, LOW, HIGH);
  } else if (command == "LEFT") {
    setMotorState(LOW, HIGH, HIGH, LOW);
  } else if (command == "RIGHT") {
    setMotorState(HIGH, LOW, LOW, HIGH);
  } else if (command == "STOP") {
    setMotorState(LOW, LOW, LOW, LOW);
  }
}