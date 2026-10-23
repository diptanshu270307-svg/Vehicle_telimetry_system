# ESP32 Hardware Setup & Wiring Guide (ASEP / EDI Vehicle Integration)

This folder (`esp32_firmware/main_car_controller/`) contains the production **FreeRTOS Dual-Core ESP32 port** of the **Telemetry & Anomaly Reconstruction Engine (TARE)**.

It runs the exact same pure C11 core (`circular_queue.c`, `linked_list.c`, `detector.c`, `state_machine.c`, `publisher.c`, `security.c`, `modem_at.c`) as the desktop simulator, connected to physical sensors and relays through [`hal_esp32.h`](main_car_controller/hal_esp32.h).

---

## 1. Pin Mapping (`src/config.h` & `hal_esp32.h`)

| Subsystem | Hardware Component | ESP32 Pin | Electrical / Fail-Safe Notes |
| :--- | :--- | :--- | :--- |
| **IMU (250 Hz)** | MPU6050 `SDA` / `SCL` | `GPIO 21` / `GPIO 22` | I2C Fast Mode (`400 kHz`), `±16g` accel, `±2000°/s` gyro |
| **Temperature (5 Hz)** | LM35 Analog Out | `GPIO 34` (`ADC1_CH6`) | Input-only ADC1 pin ($10\text{ mV}/^\circ\text{C}$) |
| **Wheel Speed (5 Hz)** | Optocoupler / Hall Sensor | `GPIO 35` | Pulse counter (`km/h`) |
| **Relay 1: Engine Cut** | Ignition / Motor Cut Relay | `GPIO 25` | **NC (Normally Closed)** — `0 = RUN`, `1 = CUT` (only when `speed <= 3 km/h`) |
| **Relay 2: Siren** | 12V Piezo Siren Relay | `GPIO 27` | **NO (Normally Open)** — `0 = OFF`, `1 = ON` |
| **Relay 3: Hazard Lights** | Indicator Flasher Relay | `GPIO 26` | **NO (Normally Open)** — `0 = OFF`, `1 = ON` |
| **Relay 4: Door Lock** | Central Lock Solenoid | `GPIO 14` | **Fail-Safe UNLOCK (`0`)** on crash so rescuers can open doors; `1 = LOCK` on theft |
| **4G + GNSS Modem** | SIM7670C `RXD` / `TXD` | `GPIO 16` / `GPIO 17` | `UART2` @ `115200 8N1` (`AT+CGATT=1`, `AT+CGNSSINFO`, `AT+HTTPINIT`) |
| **LoRa SOS Fallback** | SX1278 433 MHz Module | `GPIO 5, 18, 19, 23, 2` | SPI (`CS=5`, `SCK=18`, `MISO=19`, `MOSI=23`, `RST=2`) |

> **Need to change a pin to match your existing car breadboard?**
> Edit the `CFG_PIN_*` constants in [`config.h`](main_car_controller/config.h) (or in [`src/config.h`](../src/config.h) and run `make sync-esp32`).

---

## 2. Dual-Core FreeRTOS Task Architecture

```
+-------------------------------------------------------------------+
|                        ESP32 (Dual-Core SoC)                      |
|                                                                   |
|  Core 1 (Priority 5, 4 ms Period = 250 Hz)                        |
|  [SensorDetectorTask]                                             |
|    |-- Reads MPU6050 I2C (250 Hz) + LM35 ADC1 (5 Hz)              |
|    |-- Enqueues TelemetryPacket into Static CircularQueue (256)   |
|    |-- Runs O(1) Multi-Signal Detector (424 ns latency)           |
|    |-- Steps Safe Relay State Machine (NORMAL->SUSPECTED->...)    |
|    |-- Feeds 3.0s Hardware Task Watchdog (esp_task_wdt)           |
|    +-- Pushes AlertNode to FreeRTOS Queue (q_alerts) -----------+ |
|                                                                 | |
|  Core 0 (Priority 2, 200 ms Period = 5 Hz)                      | |
|  [CommsTask] <--------------------------------------------------+ |
|    |-- Inserts alerts into Static Pool Outbox (64 nodes, 0 malloc)|
|    |-- Signs payload with Device HMAC-SHA256 + 64-bit Nonce       |
|    |-- Transmits via SIM7670C 4G HTTPS POST + LoRa SOS Fallback   |
|    +-- Batches LittleFS (/outbox.dat) writes to prevent flash wear|
+-------------------------------------------------------------------+
```

---

## 3. Using 1 ESP32 vs. 2 ESP32s

- **Single ESP32 Mode (Default)**:
  Flash `esp32_firmware/main_car_controller/main_car_controller.ino` onto one ESP32. Core 1 handles 250 Hz sensing + relays, and Core 0 handles SIM7670C / LoRa / LittleFS outbox.
- **Dual ESP32 Mode (ESP32 #1 In-Car Controller + ESP32 #2 Telemetry/Gateway Node)**:
  Connect ESP32 #1 `UART2 TX (GPIO 17)` $\rightarrow$ ESP32 #2 `UART RX (GPIO 16)` (and common `GND`). ESP32 #1 runs the 250 Hz crash detector and physical relays with zero network delay, while streaming signed JSON/AT frames over UART to ESP32 #2 for Wi-Fi / 4G / LoRa cloud upload.

---

## 4. Flashing Instructions (Arduino IDE)

1. Install **ESP32 Board Support** (`esp32 by Espressif Systems`) in Arduino IDE.
2. Open [`esp32_firmware/main_car_controller/main_car_controller.ino`](main_car_controller/main_car_controller.ino).
3. Select **Board**: `ESP32 Dev Module`, **Upload Speed**: `921600`, **Partition Scheme**: `Default 4MB with spiffs/LittleFS`.
4. Keep the vehicle flat and stationary for the first **2 seconds** after power-on so `hal_imu_calibrate()` automatically zeroes out accelerometer/gyro mounting offsets and removes the $1g$ gravity vector.
