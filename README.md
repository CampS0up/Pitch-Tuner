# **ESP32 Vocal Pitch Tuner – Firmware + Android App**

This project implements a **real-time vocal pitch tuner** using:

* **ESP32 + MAX9814 microphone module**
* **YIN pitch detection algorithm (optimized + band-limited)**
* **35 kHz I2S ADC sampling for clean audio input**
* **BLE UART service** for streaming pitch data to an Android app
* **Android app** that displays note, frequency, cents error, confidence, and tuning stability
* Optional **reference tone playback** via buzzer

The system provides **fast**, **stable**, and **low-latency pitch tracking** suitable for singing feedback, wearable devices, and other musical applications.

---

## 🎤 **Features**

### **ESP32 Firmware**

* MAX9814 microphone input via **I2S ADC @ 35 kHz**
* YIN-based real-time pitch detection
* Median smoothing + calibration offsets
* BLE notifications (Nordic UART-style)
* 60 ms pitch update interval
* Automatic BLE advertising recovery
* Reference tone system:

  * `REF ON` → play nearest musical note
  * `PLAY NOTE=C4` or `PLAY F=440`
* Adjustable concert pitch (`SET A4=442`)

### **Android App**

* Scans only for **ESP32-YinPitch-44k** devices (filtered)
* Displays:

  * Current note
  * Frequency
  * Cents offset
  * Confidence / probability
* Sends commands (start/stop, play, mute, reference tone)
* Accuracy Log Mode:

  * Collects cents error values
  * Computes average error, maximum deviation, and stability percentage
  * Shows most recent raw JSON packet
* Built using **Kotlin + BLE GATT**

---

## 🛠️ **Hardware Setup (ESP32 + MAX9814)**

### **Microphone Wiring**

| MAX9814 Pin | ESP32 Pin          |
| ----------- | ------------------ |
| **VDD**     | 3.3V               |
| **GND**     | GND                |
| **OUT**     | GPIO 34 (ADC1_CH6) |

### **Buzzer Wiring**

| Buzzer Pin | ESP32 Pin |
| ---------- | --------- |
| **+**      | GPIO 13   |
| **–**      | GND       |

---

## 🔧 **Arduino Setup**

### **Required Tools**

1. Install **ESP32 Arduino Core v2.0+**
   *Boards Manager → Search “esp32” → Install*

2. Required libraries:

```
NimBLE-Arduino
```

3. Recommended Board Settings:

```
Tools → Board → ESP32 Dev Module
Upload Speed: 921600 (optional)
Flash Size: 4 MB
```

---

## 📡 **BLE Service Description**

The ESP32 exposes a **Nordic UART-style BLE service**:

| Purpose            | UUID                                   |
| ------------------ | -------------------------------------- |
| **Service**        | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| **RX (App → ESP)** | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| **TX (ESP → App)** | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

---

## 💬 **BLE Commands**

Send these as plain text strings:

| Command        | Description                    |
| -------------- | ------------------------------ |
| `START`        | Begin pitch streaming          |
| `STOP`         | Pause pitch detection          |
| `MUTE`         | Silence buzzer                 |
| `PLAY NOTE=C4` | Play musical note              |
| `PLAY F=440`   | Play specific frequency        |
| `REF ON`       | Enable reference note tracking |
| `REF OFF`      | Turn off reference tone        |
| `SET A4=442`   | Adjust concert tuning          |
| `PING`         | ESP32 returns a JSON echo      |

---

## 📤 **BLE JSON Output Format**

Typical packet sent from ESP32:

```json
{
  "f0": 293.66,
  "note": "D4",
  "cents": -12.5,
  "rms": 0.136,
  "prob": 0.82
}
```

| Field   | Meaning                            |
| ------- | ---------------------------------- |
| `f0`    | Detected pitch (after calibration) |
| `note`  | Nearest musical note               |
| `cents` | Sharp/flat difference in cents     |
| `rms`   | Signal energy (volume)             |
| `prob`  | YIN confidence                     |

---

## 📱 **Android App Setup**

### **Requirements**

* Android Studio **Flamingo** or newer
* Minimum SDK: **26**

### **Required Permissions**

* `BLUETOOTH_SCAN`
* `BLUETOOTH_CONNECT`
* `ACCESS_FINE_LOCATION`

### **Build Instructions**

1. Open project folder in Android Studio
2. Select a physical Android device (BLE required)
3. Build → **Make Project**
4. Run → **Run App**
5. Tap **Scan**
6. Select **ESP32-YinPitch-44k**

---

## 📊 **Accuracy Logging**

When tapping **Start Accuracy Log**:

* Clears previously collected errors
* Records absolute deviation from target pitch (in cents)
* On stopping, computes:

| Metric            | Description                   |
| ----------------- | ----------------------------- |
| **Average Error** | Mean deviation in cents       |
| **Max Error**     | Worst deviation observed      |
| **Stability %**   | % of samples within ±10 cents |

Useful for vocal warmups, practice sessions, and evaluating pitch control.

---

## 🔌 **Power Usage**

Average ESP32 + MAX9814 consumption:

* **150 mA @ 5V**
* ~**0.75 W** continuous draw

### **Battery Runtime Estimates**

| Power Bank     | Estimated Runtime |
| -------------- | ----------------- |
| **10,000 mAh** | ~39 hours         |
| **20,000 mAh** | ~79 hours         |

---

## 🔍 **Troubleshooting**

### **ESP32 Not Advertising**

* Ensure this is executed after disconnect:

  ```cpp
  NimBLEDevice::startAdvertising();
  ```
* Check serial output:

  ```
  [BLE] Advertising started
  ```

### **App Shows “Write failed”**

* Confirm RX characteristic uses:

  ```cpp
  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  ```

### **Pitch Always Flat/Sharp**

* Adjust global calibration:

  ```cpp
  #define CALIBRATION_SEMITONES -3.7f
  ```
* Use per-range corrections (e.g., C2 low offset, B4 high offset)

### **Scanner Shows Too Many Devices**

Add app-side filter:

```kotlin
if (!name.contains("ESP32") && !name.contains("YinPitch")) return
```

---

## 📄 **License**

**MIT License** — free to use, modify, and distribute.

---

## 🙌 **Credits**

* Developed by **Campbell Brown**
* Pitch engine based on the **YIN algorithm**, optimized for ESP32
* BLE transport using **NimBLE-Arduino**

---

If you want a **PDF version**, a **GitHub badge setup**, or a **diagram section**, I can generate those too!
