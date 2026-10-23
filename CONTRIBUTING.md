# Contributing to Telemetry & Anomaly Reconstruction Engine (TARE)

Thank you for your interest in contributing to **TARE**! This project is written in strict, zero-dependency **C11** so the exact same core algorithms run on both a desktop simulator (`sim` HAL backend) and physical **ESP32 FreeRTOS** hardware (`esp32` HAL backend).

---

## Architectural Invariants

Before opening a Pull Request, please ensure your changes respect these core constraints:

1. **Zero Hardware `#ifdef`s in Core Engine (`src/*.c`)**:
   - All hardware interaction must go through [`src/hal.h`](src/hal.h) (`hal_imu`, `hal_temp`, `hal_speed_gps`, `hal_relay`, `hal_modem`, `hal_lora`, `hal_storage`, `hal_time`).
   - Core modules (`circular_queue`, `linked_list`, `detector`, `state_machine`, `publisher`, `security`, `modem_at`) must compile identically on desktop and ESP32.
2. **Zero Dynamic Heap Allocation (`malloc`/`free`) in Hot Paths**:
   - The black-box ring buffer uses a fixed static array (`storage[256]`).
   - The severity-ordered outbox uses a static node pool (`pool[64]`) with an $O(1)$ free-list.
3. **Hardware-Free Verifiability**:
   - Every feature must be testable on a laptop with zero hardware attached via `make test` and `make demo`.

---

## Development Workflow

### 1. Build and Run All Tests
```bash
make clean && make all && make test
```
All 7 test suites must pass with zero compiler warnings (`-Wall -Wextra -std=c11`):
- `test_circular_queue` — Static ring buffer wraparound & pre-crash snapshot extraction.
- `test_linked_list` — Malloc-free static pool (`64` nodes), severity ordering, and batched flash-wear persistence.
- `test_detector` — Boot IMU calibration, gravity removal, pothole rejection, hysteresis, and rollover persistence.
- `test_replay` — 250 Hz CSV trace replay harness & confusion matrix (`Precision = 1.000, Recall = 1.000`).
- `test_state_machine` — `NORMAL -> SUSPECTED -> CONFIRMED -> LOCKDOWN`, speed-safe engine cut (`<= 3.0 km/h`), PIN override (`1234`), and hardware watchdog.
- `test_security_modem` — Pure C11 SHA-256 + HMAC-SHA256, 64-bit nonce anti-replay cache, bare `UNLOCK` rejection, and SIM7670C AT state machine.
- `test_benchmark` — 100,000-iteration latency & cycle benchmark verifying the 250 Hz real-time budget.

### 2. Synchronize ESP32 Firmware
Whenever you modify any core `.c` or `.h` file in `src/`, synchronize the Arduino/ESP32 sketch directory before committing:
```bash
make sync-esp32
```

### 3. Test the Live Dashboard (`TARE // MX-PRO`)
```bash
./run_demo.sh
```
Verify that the SSE stream (`/api/stream`), differential (`Δ`) oscilloscope charts, forensic timeline scrubber, and CSV export (`/api/blackbox.csv`) work cleanly in your browser.
