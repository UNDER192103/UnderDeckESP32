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
```