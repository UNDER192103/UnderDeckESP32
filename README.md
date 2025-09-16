# ESP32 WebSocket and Command Handler

This project provides a comprehensive solution for controlling an ESP32 via a WebSocket connection. It allows for dynamic pin configuration, saving Wi-Fi credentials to flash memory, and handling real-time sensor and button events. The core functionality is based on JSON commands, which can be sent via a WebSocket server or directly through a USB serial connection.

## Features

*   **WebSocket Client**: Connects to a secure WebSocket server (`wss://`) for bidirectional, real-time communication.
*   **Dynamic Command Handling**: Parses JSON commands to control LEDs, read analog sensors, and manage a servo motor.
*   **Persistent Wi-Fi Credentials**: Saves Wi-Fi SSID and password to the ESP32's flash memory using the `Preferences` library.
*   **Access Point (AP) Mode**: Switches to a default Access Point (`UnderDeck`) if no saved Wi-Fi credentials are found or the connection fails.
*   **Dynamic MAC Address Integration**: Obtains the ESP32's unique MAC address and appends it to the WebSocket URL for device identification.
*   **Sensor Event Publishing**: Monitors a button, potentiometer, and proximity sensor, automatically publishing JSON events to the WebSocket server when their state or value changes.
*   **Activity Indicator**: Blinks the built-in LED on the ESP32 to provide visual feedback whenever a command is received.
*   **Debug Mode**: A configurable debug mode can be enabled or disabled to toggle detailed serial logging. This setting is saved to flash memory.
*   **Serial Control**: All commands can also be sent via the USB serial connection for easy testing and debugging.

## Requirements

*   **ESP32 DEVKITV1** board
*   **Arduino IDE** with ESP32 board support installed
*   **Libraries**:
    *   `WebSockets` (by Markus Sattler)
    *   `ArduinoJson`
    *   `ESP32Servo`
    *   `Preferences` (built-in for ESP32)

You can install these libraries using the Arduino IDE's Library Manager (`Sketch > Include Library > Manage Libraries...`).

## Setup and Installation

1.  **Open the Arduino IDE** and load the provided `.ino` file.
2.  **Ensure all required libraries are installed** via the Library Manager.
3.  **Connect your ESP32** board to your computer via USB.
4.  **Select the correct board** (`ESP32 DEVKITV1`) and COM port from the `Tools` menu.
5.  **Modify the WebSocket URL**:
    *   The code is pre-configured to connect to `wss://wss-global.undernouzen.shop/esp32?`.
    *   Ensure your server can handle this connection format.
6.  **Upload the code** to the ESP32.

## Initial Wi-Fi Configuration

After the initial upload, the ESP32 will either:
*   Connect to a previously saved network.
*   Start in Access Point (AP) mode with the SSID `UnderDeck` and password `12345678`.

To configure Wi-Fi credentials for the first time, use the serial monitor:
1.  Open the Arduino Serial Monitor (`Ctrl + Shift + M`).
2.  Set the baud rate to `115200`.
3.  Send the following JSON command to save your network details:

**Command:**
```json
{"action":"set_wifi", "ssid":"YourNetworkSSID", "password":"YourNetworkPassword"}