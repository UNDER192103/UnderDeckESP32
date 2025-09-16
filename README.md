# ESP32 WebSocket and Command Handler

This project provides a comprehensive solution for controlling an ESP32 via a WebSocket connection. It allows for dynamic pin configuration, saving Wi-Fi credentials to flash memory, and handling real-time sensor and button events. The core functionality is based on JSON commands, which can be sent via a WebSocket server or directly through a USB serial connection.

## Features

*   **WebSocket Client**: Connects to a secure WebSocket server (`wss://`) for bidirectional, real-time communication.
*   **Dynamic Command Handling**: Parses JSON commands to control LEDs, read analog sensors, and manage a servo motor.
*   **Persistent Wi-Fi Credentials**: Saves Wi-Fi SSID and password to the ESP32's flash memory using the `Preferences` library.
*   **Access Point (AP) Mode**: Switches to a default Access Point (`UnderDeck`) if no saved Wi-Fi credentials are found or the connection fails.
*   **Dynamic MAC Address Integration**: Obtains the ESP32's unique MAC address and appends it to the WebSocket URL for device identification.
*   **Sensor Monitor**: Enables or disables continuous monitoring of button, potentiometer, and touch sensors via WebSocket commands.
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

# ESP32 JSON Command API

This documentation details the JSON commands that can be sent to the ESP32 via WebSocket or the Serial Monitor for device configuration and hardware control.

## Device Configuration Commands

---

### `set_wifi`
> Saves Wi-Fi credentials to flash memory. A device restart is required for the changes to take effect.

**Parameters:**

| Parameter | Type   | Description             |
|:----------|:-------|:------------------------|
| `ssid`    | string | The network SSID.       |
| `password`| string | The network password.   |

**Command Example:**
```json
{"action":"set_wifi", "ssid":"YourNetworkSSID", "password":"YourNetworkPassword"}
```

**Success Response:**
```json
{"status":"success","message":"WiFi credentials saved. Restart to reconnect."}
```

---

### `reboot_wifi`
> Disconnects from the current network and attempts to reconnect using saved credentials. If none are found, it switches to AP mode.

**Command Example:**
```json
{"action":"reboot_wifi"}
```

**Success Response:**
```json
{"status":"success","message":"Attempting to restart WiFi connection."}
```

---

### `get_mac`
> Requests the unique MAC address of the ESP32.

**Command Example:**
```json
{"action":"get_mac"}
```

**Success Response:**
```json
{"status":"success","mac_address":"AABBCCDDEEFF"}
```
*(The format is without colons)*

---

### `get_status`
> Requests the ESP32 wifi status.

**Command Example:**
```json
{"action":"get_status"}
```

**Success Response:**
```json
{"status":"success","connected": true,"message": "Connected to WiFi"}
```

---

### `get_device_info`
> Requests the ESP32 info.

**Command Example:**
```json
{"action":"get_device_info"}
```

**Success Response:**
```json
{"status":"success","hostname": "esp32-xxxxxx","mac_address": "AABBCCDDEEFF"}
```

---

### `set_debug`
> Enables (`1`) or disables (`0`) detailed serial debug output. The setting is saved to flash.

**Parameters:**

| Parameter | Type | Description                    |
|:----------|:-----|:-------------------------------|
| `mode`    | int  | `1` to enable, `0` to disable. |

**Command Example:**
```json
{"action":"set_debug", "mode":1}
```

**Success Response:**
```json
{"status":"success","message":"Debug mode changed to 1"}
```

---

### `enable_monitor`
> Enables continuous monitoring for a specific sensor type on a specific pin.

**Parameters:**

| Parameter | Type   | Description                                                  |
|:----------|:-------|:-------------------------------------------------------------|
| `pin`     | int    | The GPIO pin number.                                         |
| `type`    | string | The sensor type: `"button"`, `"potentiometer"`, or `"touch"`. |

**Command Example:**
```json
{"action":"enable_monitor", "pin":14, "type":"button"}
```

**Success Response:**
```json
{"status":"success","message":"Enabled monitoring for button on pin 14"}
```

---

### `disable_monitor`
> Disables continuous monitoring for a specific sensor type on a specific pin.

**Parameters:**

| Parameter | Type   | Description                                                  |
|:----------|:-------|:-------------------------------------------------------------|
| `pin`     | int    | The GPIO pin number.                                         |
| `type`    | string | The sensor type: `"button"`, `"potentiometer"`, or `"touch"`. |

**Command Example:**
```json
{"action":"disable_monitor", "pin":14, "type":"button"}
```

**Success Response:**
```json
{"status":"success","message":"Disabled monitoring for button on pin 14"}
```

## Hardware Control Commands

---

### `led`
> Controls an LED connected to a specific pin.

**Parameters:**

| Parameter | Type   | Description          |
|:----------|:-------|:---------------------|
| `pin`     | int    | The GPIO pin number. |
| `state`   | string | `"on"` or `"off"`.   |

**Command Example (Turn On):**
```json
{"action":"led", "pin":2, "state":"on"}
```

**Success Response:**
```json
{"status":"success","message":"LED turned on at pin 2"}
```

---

### `read_analog`
> Reads the analog value from a specific pin.

**Parameters:**

| Parameter | Type | Description                |
|:----------|:-----|:---------------------------|
| `pin`     | int  | The analog GPIO pin number.|

**Command Example:**
```json
{"action":"read_analog", "pin":34}
```

**Success Response:**
```json
{"status":"success","message":"Analog value read at pin 34"}
```
*(The actual value is sent as a `sensor_update` event)*

---

### `servo`
> Moves a servo motor connected to a PWM pin to a specified position in degrees.

**Parameters:**

| Parameter  | Type | Description                       |
|:-----------|:-----|:----------------------------------|
| `pin`      | int  | The PWM GPIO pin number.          |
| `position` | int  | The position in degrees (0-180).  |

**Command Example:**
```json
{"action":"servo", "pin":13, "position":90}
```

**Success Response:**
```json
{"status":"success","message":"Servo moved to 90 degrees at pin 13"}
```

## Event-Driven Sensor Updates
> These are JSON events sent by the ESP32 when the state of a monitored sensor changes.

---

### `sensor_update`
> Sent when a change is detected in a sensor enabled via `enable_monitor`.

**Event Fields:**

| Field     | Type   | Description                                                                     |
|:----------|:-------|:--------------------------------------------------------------------------------|
| `action`  | string | `"sensor_update"`                                                               |
| `type`    | string | The sensor type: `"button"`, `"potentiometer"`, or `"touch"`.                   |
| `pin`     | int    | The GPIO pin number.                                                            |
| `value`   | int    | The new sensor value (state for button, analog reading for potentiometer and touch). |
| `timestamp`| long  | Timestamp in milliseconds.                                                      |

**Event Examples:**

*   **Button Press:**
    ```json
    {"action":"sensor_update","type":"button","pin":14,"value":0,"timestamp":1234567}
    ```

*   **Potentiometer Change:**
    ```json
    {"action":"sensor_update","type":"potentiometer","pin":34,"value":2048,"timestamp":1234567}
    ```

*   **Touch Sensor Activation:**
    ```json
    {"action":"sensor_update","type":"touch","pin":4,"value":50,"timestamp":1234567}
    ```