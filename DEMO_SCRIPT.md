# One-Page Live Demo Script (Hardware-Free Mode)

This guide walks through a complete 3-minute demonstration of the **Telemetry & Anomaly Reconstruction Engine** on any laptop with **zero hardware attached**.

---

## 0. Launch the Demo (10 seconds)

Run a single command from the repository root:

```bash
make demo
```

* **What happens:** Compiles the pure C11 engine (`sim` HAL backend), prints the boot-time MPU6050 bias calibration and gravity removal report, launches the embedded SSE + Mock Cloud server on `http://127.0.0.1:8080/`, opens the dashboard automatically, and starts the deterministic `--auto-demo --seed 42` story.

---

## 1. Show Real-Time Internals & Ring Buffer (0:00 – 0:30)

* **Where to look:** **Internals: Circular Queue Black-Box & O(1) Detector** panel (middle-left).
* **What the audience sees:**
  * The 32-segment **Circular Queue Ring Buffer** filling in real time with `head` (amber) and `tail` (red) cursors advancing in $\mathcal{O}(1)$ time, and the **Overwrites** counter ticking once full (`128/128` packets).
  * The **250 Hz MPU6050 waveform** and rolling 16-sample `mean`, `variance`, and `Z-score`.

---

## 2. Pothole Rejection vs. Real Crash Fusion (0:30 – 1:15)

1. **Click `Pothole (Noise Filter)`** in the top toolbar (or wait for `t = 3.0s` in auto-demo).
   * **What the audience sees:** A sharp vertical acceleration spike appears on the plot, the Z-score jumps above `3.5`, and the **Detector Explainability Banner** reads:
     `FILTERED_POTHOLE: single-spike |a|=... rejected`
   * **Key takeaway:** Single-sample road bumps do **not** trigger false crash alerts because there is no sustained duration or speed drop.
2. **Click `Collision`** in the top toolbar.
   * **What the audience sees:**
     * Detector fuses high $|a| > 25\text{ m/s}^2$ with a sudden speed drop ($\Delta v$) and fires `COLLISION (CRITICAL)`.
     * **Virtual GNSS Map** drops a red `CRASH SITE` pin and draws a dashed red dispatch line to `HOSPITAL (AIIMS Trauma Center)`.

---

## 3. Safe Relay State Machine & PIN Override (1:15 – 2:00)

* **Where to look:** **Safe Relay State Machine & Virtual Actuators** panel (top-right).
* **What the audience sees:**
  1. Immediately after a crash, the state machine enters **`2. SUSPECTED`** (flashing amber)—activating **only** the Siren (`GPIO 27`) and Hazard Lights (`GPIO 26`) while keeping the Engine (`GPIO 25`) allowed to run during the 3-second confirmation window.
  2. **Demo False-Trigger Cancel:** Click **`Cancel / Override State (PIN)`** (with PIN `1234`) while in `SUSPECTED`. The system cancels the alarm and returns to `1. NORMAL`.
  3. **Demo Confirmed Crash Interlock:** Click **`Rollover`** and let the 3-second confirmation timer expire into **`3. CONFIRMED`**. Notice that while vehicle speed $> 3.0\text{ km/h}$, engine cut is **deferred** (`interlock_wait_low_speed`) so a sliding car never loses power steering/brakes; once speed drops $\le 3.0\text{ km/h}$, `GPIO 25 Engine Cut-Off` turns red and `GPIO 14 Door Locks` unlock for rescue.

---

## 4. Network Drop, Severity Outbox & Batched Flash Wear (2:00 – 2:30)

1. **Click `Toggle 4G Network`** so the top badge shows **`4G DOWN`**.
2. **Click `Overheating`**, **`Hard Brake`**, and **`Collision`** in sequence.
   * **What the audience sees:**
     * In the **Alert Outbox Linked List**, `COLLISION (CRITICAL)` is automatically spliced **ahead** of `OVERHEAT (HIGH)` and `HARD_BRAKE (HIGH)` via $\mathcal{O}(n)$ severity-ordered insertion.
     * While 4G is down, the **LoRa 433MHz** transceiver transmits emergency SOS beacons (`LORA_SOS|...`) and retry timers double with exponential backoff (`2s -> 4s -> 8s`).
     * The **Batched Flash Writes** counter shows multiple RAM mutations consolidated into periodic LittleFS/disk writes.
3. **Click `Toggle 4G Network`** back to **`4G UP`** and watch the outbox drain in priority order (`CRITICAL` first).

---

## 5. Black-Box Forensic Scrubber & Cyberattack Defense (2:30 – 3:00)

1. **Check `Freeze Live Scrubber`** under the GPS map and drag the **Black-Box Forensic Replay Scrubber** slider left and right to replay the pre-crash window frame-by-frame. Click **`Export Black-Box CSV`** to download `blackbox_forensics.csv`.
2. In the **Mock Cloud Remote Link** panel (bottom-right):
   * Click **`Cloud Signed LOCKDOWN`** $\rightarrow$ `VERIFIED_OK` (car enters `4. LOCKDOWN`, doors lock).
   * Click **`Simulate Replay Attack`**, **`Simulate Bad HMAC`**, or **`Attempt Bare UNLOCK`** $\rightarrow$ watch the security audit banner turn red (`REJECTED_REPLAY`, `REJECTED_SIG`, `bare UNLOCK endpoint forbidden`) and the `Blocked` counter increment!
