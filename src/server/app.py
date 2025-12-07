from flask import Flask, render_template, jsonify, request
import paho.mqtt.client as mqtt
import json
from datetime import datetime

app = Flask(__name__)

# ===========================
# KONFIGURASI MQTT
# ===========================
MQTT_BROKER = "localhost"  # Mosquitto running on this laptop
MQTT_PORT = 1883
MQTT_TOPIC_MOVE = "rc/move"

# MQTT Client
mqtt_client = mqtt.Client()
mqtt_connected = False

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print(f"[MQTT] Connected to broker at {MQTT_BROKER}:{MQTT_PORT}")
        mqtt_connected = True
    else:
        print(f"[MQTT] Connection failed with code {rc}")
        mqtt_connected = False

def on_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    print("[MQTT] Disconnected from broker")

mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect

# Command history
command_history = []

def log_command(command):
    """Log command to history"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    command_history.append({"time": timestamp, "command": command})
    # Keep only last 10 commands
    if len(command_history) > 10:
        command_history.pop(0)

# ===========================
# ROUTES
# ===========================

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status')
def status():
    """Get current status"""
    return jsonify({
        "mqtt_connected": mqtt_connected,
        "broker": f"{MQTT_BROKER}:{MQTT_PORT}",
        "history": command_history[-5:]  # Last 5 commands
    })

@app.route('/api/move', methods=['POST'])
def move():
    """Send movement command via MQTT"""
    data = request.get_json()
    command = data.get('command', 'STOP')
    
    if mqtt_connected:
        mqtt_client.publish(MQTT_TOPIC_MOVE, command)
        log_command(command)
        print(f"[SENT] {command}")
        return jsonify({"success": True, "command": command})
    else:
        return jsonify({"success": False, "error": "MQTT not connected"}), 503

# ===========================
# MAIN
# ===========================

if __name__ == '__main__':
    print("\n========================================")
    print("   RC CAR CONTROL SERVER")
    print("========================================\n")
    
    # Connect to MQTT Broker
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
    except Exception as e:
        print(f"[MQTT] Failed to connect: {e}")
        print("[MQTT] Make sure Mosquitto broker is running!")
    
    print("\n[SERVER] Starting Flask server...")
    print("[SERVER] Open http://localhost:5000 in your browser")
    print("========================================\n")
    
    app.run(host='0.0.0.0', port=5000, debug=True)
