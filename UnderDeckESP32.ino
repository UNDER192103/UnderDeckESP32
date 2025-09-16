#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// Define the WebSocket server address
const char* websocket_server_host = "wss-global.undernouzen.shop";
const uint16_t websocket_server_port = 443; // Standard port for WSS
const char* websocket_path = "/ESP32?";

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
int sensorPin = -1;
bool is_servo_attached = false;

// Debug property
int debug_mode = 0; // 0 = disabled, 1 = enabled
const int builtInLed = 2; // Built-in LED of the ESP32 board

// Button monitoring variables
const int buttonPin = 14; // Example pin for the button
int lastButtonState = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;

// Potentiometer monitoring variables
const int potPin = 34; // Example pin for the potentiometer (analog)
int lastPotValue = 0;
const int potThreshold = 50; // Minimum change to trigger an event

// Proximity/Presence sensor monitoring variables
const int proximityPin = 13; // Example pin for proximity sensor (digital)
int lastProximityState = HIGH;

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
void checkButton();
void checkPotentiometer();
void checkProximitySensor();
void sendButtonEventToWebSocket();
void sendPotentiometerEventToWebSocket(int value);
void sendProximityEventToWebSocket(int state);

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

  // Configure the button pin with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  // Configure the proximity sensor pin with internal pull-up resistor
  pinMode(proximityPin, INPUT_PULLUP);

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
    // Check if MAC Address was obtained and WebSocket connection is established
    if (current_mac_address.length() > 0 && !ws_connected) {
        connectToWebSocket();
    }
    webSocketClient.loop();
  }

  // Handle commands via Serial
  processSerial();

  // Check all sensors
  checkButton();
  checkPotentiometer();
  checkProximitySensor();

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

    // Get MAC Address only after successful connection
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
  
  // In AP mode, MAC Address is available
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
  int mode = preferences.getInt("debug_mode", 0); // Default value 0
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

    Serial.print("Attempting to connect to wss://");
    Serial.print(websocket_server_host);
    Serial.println(url);

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
  } else if (strcmp(action, "read_sensor") == 0) {
    const int pin = doc["pin"];
    if (pin != sensorPin) {
      pinMode(pin, INPUT);
      sensorPin = pin;
    }
    int sensorValue = analogRead(sensorPin);
    responseDoc["sensor_value"] = sensorValue;
    responseDoc["pin"] = sensorPin;
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
  if (debug_mode == 1) Serial.println(response);
  if (!isSerial) {
    webSocketClient.sendTXT(response);
  }
}

void blinkBuiltInLed(int duration) {
  digitalWrite(builtInLed, HIGH);
  delay(duration);
  digitalWrite(builtInLed, LOW);
}

void checkButton() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastButtonState) {
      lastButtonState = reading;
      if (lastButtonState == LOW) { // Button pressed
        if (debug_mode == 1) Serial.println("Button pressed. Sending event to WebSocket.");
        sendButtonEventToWebSocket();
      }
    }
  }
}

void checkPotentiometer() {
  int currentPotValue = analogRead(potPin);
  if (abs(currentPotValue - lastPotValue) > potThreshold) {
    lastPotValue = currentPotValue;
    if (debug_mode == 1) Serial.println("Potentiometer value changed. Sending event to WebSocket.");
    sendPotentiometerEventToWebSocket(currentPotValue);
  }
}

void checkProximitySensor() {
  int currentProximityState = digitalRead(proximityPin);
  if (currentProximityState != lastProximityState) {
    lastProximityState = currentProximityState;
    if (debug_mode == 1) Serial.println("Proximity sensor state changed. Sending event to WebSocket.");
    sendProximityEventToWebSocket(currentProximityState);
  }
}

void sendButtonEventToWebSocket() {
  if (ws_connected) {
    StaticJsonDocument<128> eventDoc;
    eventDoc["action"] = "button_press";
    eventDoc["pin"] = buttonPin;
    eventDoc["timestamp"] = millis();
    String json_string;
    serializeJson(eventDoc, json_string);
    webSocketClient.sendTXT(json_string);
    blinkBuiltInLed(50);
  } else {
    if (debug_mode == 1) Serial.println("WebSocket not connected. Cannot send button event.");
  }
}

void sendPotentiometerEventToWebSocket(int value) {
  if (ws_connected) {
    StaticJsonDocument<128> eventDoc;
    eventDoc["action"] = "potentiometer_change";
    eventDoc["pin"] = potPin;
    eventDoc["value"] = value;
    eventDoc["timestamp"] = millis();
    String json_string;
    serializeJson(eventDoc, json_string);
    webSocketClient.sendTXT(json_string);
    blinkBuiltInLed(50);
  } else {
    if (debug_mode == 1) Serial.println("WebSocket not connected. Cannot send potentiometer event.");
  }
}

void sendProximityEventToWebSocket(int state) {
  if (ws_connected) {
    StaticJsonDocument<128> eventDoc;
    eventDoc["action"] = "proximity_change";
    eventDoc["pin"] = proximityPin;
    eventDoc["state"] = state == LOW ? "detected" : "clear";
    eventDoc["timestamp"] = millis();
    String json_string;
    serializeJson(eventDoc, json_string);
    webSocketClient.sendTXT(json_string);
    blinkBuiltInLed(50);
  } else {
    if (debug_mode == 1) Serial.println("WebSocket not connected. Cannot send proximity event.");
  }
}
