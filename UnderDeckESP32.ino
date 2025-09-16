#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <vector>

// Define the WebSocket server address
const char* websocket_server_host = "wss-global.undernouzen.shop";
const uint16_t websocket_server_port = 443; // Standard port for WSS
const char* websocket_path = "/ESP32UND?";

// Object to store preferences in flash memory
Preferences preferences;

// Objects for communication and control
WebSocketsClient webSocketClient;
Servo myServo;
String current_mac_address;
bool ws_connected = false;

// Dynamic pin variables
int ledPin = -1;
int servoPin = -1;
bool is_servo_attached = false;

// Debug property
int debug_mode = 0; // 0 = disabled, 1 = enabled
const int builtInLed = 2; // Built-in LED of the ESP32 board

// Potentiometer monitoring variables
const int potThreshold = 50; // Minimum change to trigger an event

// Touch sensor monitoring variables
const int touchThreshold = 40; // Minimum value to detect a touch

// Button monitoring variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Structure to hold sensor information for monitoring
struct MonitoredSensor {
  int pin;
  String type;
  int lastValue;
};

// Vector to hold the list of active sensors
std::vector<MonitoredSensor> activeSensors;

// Function prototypes
void startWiFi(const char* ssid, const char* password);
void startWiFiAP();
void saveWiFiCredentials(const char* ssid, const char* password);
void loadWiFiCredentials(String &ssid, String &password);
void saveDebugMode(int mode);
int loadDebugMode();
String getMacAddressString();
String getMacAddressNoColons();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void handleCommand(String payload, bool isSerial);
void processSerial();
void connectToWebSocket();
void blinkBuiltInLed(int duration);
void sendSensorValueToWebSocket(int pin, const char* type, int value);
void monitorActiveSensors();
void enableSensorMonitor(int pin, const char* type);
void disableSensorMonitor(int pin, const char* type);

Serial.print("{'action':'starting'}");

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Load debug mode before any conditional serial printing
  debug_mode = loadDebugMode();
  if (debug_mode == 1) {
    Serial.println("Debug mode: ENABLED");
  } else {
    Serial.println("Debug mode: DISABLED");
  }

  // Configure built-in LED
  pinMode(builtInLed, OUTPUT);
  digitalWrite(builtInLed, LOW);

  // Initialize WiFi mode
  WiFi.mode(WIFI_STA); 
  
  // Try to load credentials and connect
  String saved_ssid, saved_password;
  loadWiFiCredentials(saved_ssid, saved_password);

  if (saved_ssid.length() > 0 && saved_password.length() > 0) {
    if (debug_mode == 1) Serial.println("Saved WiFi credentials found. Attempting to connect...");
    startWiFi(saved_ssid.c_str(), saved_password.c_str());
  } else {
    Serial.println("No WiFi credentials saved. Starting in Access Point mode.");
    startWiFiAP();
  }
}

void loop() {
  // Handle WebSocket events
  if (WiFi.status() == WL_CONNECTED) {
    if (current_mac_address.length() > 0 && !ws_connected) {
        connectToWebSocket();
    }
    webSocketClient.loop();
  } else {
    ws_connected = false;
  }

  // Handle commands via Serial
  processSerial();
  
  // Monitor active sensors
  monitorActiveSensors();
  
  // A small delay to avoid overwhelming the CPU
  delay(10);
}

void startWiFi(const char* ssid, const char* password) {
  if (debug_mode == 1) Serial.println("Attempting to connect to WiFi...");
  WiFi.begin(ssid, password);

  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED && timeout_counter < 30) {
    delay(500);
    if (debug_mode == 1) Serial.print(".");
    timeout_counter++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi network!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    current_mac_address = getMacAddressNoColons(); 
    Serial.print("ESP32 MAC Address (without ':'): ");
    Serial.println(current_mac_address);
  } else {
    Serial.println("\nConnection failed. Starting in Access Point mode.");
    startWiFiAP();
  }
}

void startWiFiAP() {
  Serial.println("Starting Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("UnderDeck", "12345678"); 
  Serial.print("Access Point (AP) started with SSID: ");
  Serial.println("UnderDeck");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  current_mac_address = getMacAddressNoColons(); 
  if (debug_mode == 1) {
    Serial.print("ESP32 MAC Address (without ':'): ");
    Serial.println(current_mac_address);
  }
}

void saveWiFiCredentials(const char* ssid, const char* password) {
  preferences.begin("wifi_creds", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
  if (debug_mode == 1) Serial.println("WiFi credentials saved to memory.");
}

void loadWiFiCredentials(String &ssid, String &password) {
  preferences.begin("wifi_creds", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  preferences.end();
}

void saveDebugMode(int mode) {
  preferences.begin("system", false);
  preferences.putInt("debug_mode", mode);
  preferences.end();
  if (debug_mode == 1) Serial.println("Debug mode saved.");
}

int loadDebugMode() {
  preferences.begin("system", true);
  int mode = preferences.getInt("debug_mode", 0);
  preferences.end();
  return mode;
}

String getMacAddressString() {
  return WiFi.macAddress();
}

String getMacAddressNoColons() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  return mac;
}

void connectToWebSocket() {
  if (ws_connected) {
    webSocketClient.disconnect();
    ws_connected = false;
  }

  if (WiFi.status() == WL_CONNECTED) {
    String url = websocket_path;
    url += current_mac_address;
    if (debug_mode == 1){
      Serial.print("Attempting to connect to wss://");
      Serial.print(websocket_server_host);
      Serial.println(url);
    }

    webSocketClient.beginSSL(websocket_server_host, websocket_server_port, url.c_str());
    webSocketClient.onEvent(webSocketEvent);
    webSocketClient.setReconnectInterval(5000);
    ws_connected = true;
  } else {
    if (debug_mode == 1) Serial.println("Not connected to WiFi, cannot connect to WebSocket.");
  }
}

void processSerial() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("{") && command.endsWith("}")) {
      if (debug_mode == 1) Serial.print("JSON command received via Serial: ");
      if (debug_mode == 1) Serial.println(command);
      blinkBuiltInLed(50);
      handleCommand(command, true);
    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server!");
      ws_connected = true;
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected from server!");
      ws_connected = false;
      break;
    case WStype_TEXT: {
      String message = String((char*)payload);
      if (debug_mode == 1) Serial.print("[WS] Message received: ");
      if (debug_mode == 1) Serial.println(message);
      blinkBuiltInLed(50);
      handleCommand(message, false);
      break;
    }
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_PONG:
      break;
  }
}

void handleCommand(String payload, bool isSerial) {
  StaticJsonDocument<512> doc; 
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    String response_error = "{\"status\":\"error\", \"message\":\"Invalid JSON\"}";
    if (debug_mode == 1) Serial.println(response_error);
    if (!isSerial) {
      webSocketClient.sendTXT(response_error);
    }
    return;
  }
  
  const char* type = doc["type"];
  if (type != nullptr && strcmp(type, "esp32") == 0) {
      const char* code = doc["code"];
      if (code != nullptr && strcmp(code, "off-line") == 0) {
          if (debug_mode == 1) Serial.println("Received 'off-line' message from server. No action needed.");
          return; 
      }
  }

  const char* action = doc["action"];
  String response;
  StaticJsonDocument<128> responseDoc;
  responseDoc["status"] = "success";
  responseDoc["action"] = action;

  if (action == nullptr) {
    responseDoc["status"] = "error";
    responseDoc["message"] = "Action not specified or invalid JSON";
  } else if (strcmp(action, "set_debug") == 0) {
    int new_mode = doc["mode"];
    if (new_mode == 0 || new_mode == 1) {
      debug_mode = new_mode;
      saveDebugMode(new_mode);
      responseDoc["message"] = "Debug mode changed to " + String(new_mode);
    } else {
      responseDoc["status"] = "error";
      responseDoc["message"] = "Invalid debug mode. Use 0 or 1.";
    }
  } else if (strcmp(action, "set_wifi") == 0) {
    const char* ssid = doc["ssid"];
    const char* password = doc["password"];
    if (ssid && password) {
      saveWiFiCredentials(ssid, password);
      responseDoc["message"] = "WiFi credentials saved. Restart to reconnect.";
    } else {
      responseDoc["status"] = "error";
      responseDoc["message"] = "Missing SSID/Password parameters.";
    }
  } else if (strcmp(action, "reboot_wifi") == 0) {
    if (debug_mode == 1) Serial.println("WiFi reboot command received.");
    String saved_ssid, saved_password;
    loadWiFiCredentials(saved_ssid, saved_password);
    if (saved_ssid.length() > 0 && saved_password.length() > 0) {
        WiFi.disconnect(true);
        startWiFi(saved_ssid.c_str(), saved_password.c_str());
    } else {
        WiFi.disconnect(true);
        startWiFiAP();
    }
    responseDoc["message"] = "Attempting to restart WiFi connection.";
  } else if (strcmp(action, "get_mac") == 0) {
    responseDoc["mac_address"] = current_mac_address;
  } else if (strcmp(action, "get_status") == 0) {
    bool isConnected = WiFi.status() == WL_CONNECTED;
    responseDoc["connected"] = isConnected;
    responseDoc["message"] = isConnected ? "Connected to WiFi" : "Not connected to WiFi";
  } else if (strcmp(action, "get_device_info") == 0) {
    responseDoc["hostname"] = WiFi.getHostname();
    responseDoc["mac_address"] = current_mac_address;
  } else if (strcmp(action, "enable_monitor") == 0) {
    int pin = doc["pin"];
    const char* sensorType = doc["type"];
    if (pin >= 0 && sensorType != nullptr) {
      enableSensorMonitor(pin, sensorType);
      responseDoc["message"] = "Enabled monitoring for " + String(sensorType) + " on pin " + String(pin);
    } else {
      responseDoc["status"] = "error";
      responseDoc["message"] = "Invalid parameters for enabling monitor.";
    }
  } else if (strcmp(action, "disable_monitor") == 0) {
    int pin = doc["pin"];
    const char* sensorType = doc["type"];
    if (pin >= 0 && sensorType != nullptr) {
      disableSensorMonitor(pin, sensorType);
      responseDoc["message"] = "Disabled monitoring for " + String(sensorType) + " on pin " + String(pin);
    } else {
      responseDoc["status"] = "error";
      responseDoc["message"] = "Invalid parameters for disabling monitor.";
    }
  } else if (strcmp(action, "read_analog") == 0) {
    const int pin = doc["pin"];
    if (pin >= 0) {
      int sensorValue = analogRead(pin);
      sendSensorValueToWebSocket(pin, "analog", sensorValue);
      responseDoc["message"] = "Analog value read at pin " + String(pin);
    } else {
      responseDoc["status"] = "error";
      responseDoc["message"] = "Invalid pin specified for analog sensor.";
    }
  } else if (strcmp(action, "led") == 0) {
    const int pin = doc["pin"];
    const char* state = doc["state"];
    if (pin != ledPin) {
      pinMode(pin, OUTPUT);
      ledPin = pin;
    }
    if (strcmp(state, "on") == 0) {
      digitalWrite(ledPin, HIGH);
      responseDoc["message"] = "LED turned on at pin " + String(ledPin);
    } else if (strcmp(state, "off") == 0) {
      digitalWrite(ledPin, LOW);
      responseDoc["message"] = "LED turned off at pin " + String(ledPin);
    }
  } else if (strcmp(action, "servo") == 0) {
    const int pin = doc["pin"];
    int position = doc["position"];
    if (pin != servoPin || !is_servo_attached) {
      if (is_servo_attached) {
        myServo.detach();
      }
      myServo.attach(pin);
      servoPin = pin;
      is_servo_attached = true;
    }
    myServo.write(position);
    responseDoc["message"] = "Servo moved to " + String(position) + " degrees at pin " + String(servoPin);
  } else {
    responseDoc["status"] = "error";
    responseDoc["message"] = "Unknown command";
  }

  serializeJson(responseDoc, response);
  if (!isSerial) {
    webSocketClient.sendTXT(response);
    if (debug_mode == 1) Serial.println(response);
  } else {
    Serial.println(response);
  }
}

void blinkBuiltInLed(int duration) {
  digitalWrite(builtInLed, HIGH);
  delay(duration);
  digitalWrite(builtInLed, LOW);
}

void sendSensorValueToWebSocket(int pin, const char* type, int value) {
  if (ws_connected) {
    StaticJsonDocument<128> eventDoc;
    eventDoc["action"] = "sensor_update";
    eventDoc["type"] = type;
    eventDoc["pin"] = pin;
    eventDoc["value"] = value;
    eventDoc["timestamp"] = millis();
    String json_string;
    serializeJson(eventDoc, json_string);
    webSocketClient.sendTXT(json_string);
    Serial.println(json_string);
    blinkBuiltInLed(50);
  } else {
    if (debug_mode == 1) {
      Serial.print("WebSocket not connected. Cannot send ");
      Serial.print(type);
      Serial.println(" sensor value.");
    }
  }
}

void monitorActiveSensors() {
  for (size_t i = 0; i < activeSensors.size(); ++i) {
    MonitoredSensor& sensor = activeSensors[i];
    int currentValue = 0;
    bool valueChanged = false;

    if (sensor.type == "button") {
      int reading = digitalRead(sensor.pin);
      if (reading != sensor.lastValue) {
        lastDebounceTime = millis();
      }
      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != sensor.lastValue) {
          currentValue = reading;
          valueChanged = true;
        }
      }
    } else if (sensor.type == "potentiometer") {
      currentValue = analogRead(sensor.pin);
      if (abs(currentValue - sensor.lastValue) > potThreshold) {
        valueChanged = true;
      }
    } else if (sensor.type == "touch") {
      currentValue = touchRead(sensor.pin);
      if (abs(currentValue - sensor.lastValue) > touchThreshold) {
        valueChanged = true;
      }
    }
    
    if (valueChanged) {
      sensor.lastValue = currentValue;
      sendSensorValueToWebSocket(sensor.pin, sensor.type.c_str(), currentValue);
    }
  }
}

void enableSensorMonitor(int pin, const char* type) {
  // Check if sensor is already being monitored
  for (const auto& sensor : activeSensors) {
    if (sensor.pin == pin && sensor.type == type) {
      if (debug_mode == 1) Serial.println("Sensor is already being monitored.");
      return;
    }
  }

  // Add sensor to the list and configure pin
  MonitoredSensor newSensor;
  newSensor.pin = pin;
  newSensor.type = type;
  
  if (newSensor.type == "button") {
    pinMode(pin, INPUT_PULLUP);
    newSensor.lastValue = digitalRead(pin);
  } else if (newSensor.type == "potentiometer") {
    newSensor.lastValue = analogRead(pin);
  } else if (newSensor.type == "touch") {
    newSensor.lastValue = touchRead(pin);
  }

  activeSensors.push_back(newSensor);
  if (debug_mode == 1) {
    Serial.print("Enabled monitoring for ");
    Serial.print(type);
    Serial.print(" on pin ");
    Serial.println(pin);
  }
}

void disableSensorMonitor(int pin, const char* type) {
  for (size_t i = 0; i < activeSensors.size(); ++i) {
    if (activeSensors[i].pin == pin && activeSensors[i].type == type) {
      activeSensors.erase(activeSensors.begin() + i);
      if (debug_mode == 1) {
        Serial.print("Disabled monitoring for ");
        Serial.print(type);
        Serial.print(" on pin ");
        Serial.println(pin);
      }
      return;
    }
  }
}
