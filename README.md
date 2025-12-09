README.md
ESP32 Vocal Pitch Tuner – Firmware + Android App

This project implements a real-time vocal pitch tuner using:

ESP32 + MAX9814 microphone module

YIN pitch detection algorithm (optimized + band-limited)

35 kHz I2S ADC sampling for clean audio input

BLE UART service for streaming pitch data to an Android app

Android app that displays note, frequency, cents error, stability, and optional accuracy logging

Optional reference tone playback (buzzer)

The system provides fast, stable, and low-latency pitch tracking suitable for singing feedback, wearable devices, and music-tech experimentation.

🎤 Features
ESP32 Firmware

MAX9814 microphone input via I2S ADC @ 35 kHz

YIN-based real-time pitch detection

Median smoothing + calibration for improved accuracy

BLE notifications (“UART”-style service)

60 ms reporting interval (adjustable)

Automatic BLE advertising recovery

Reference tone system:

REF ON = play nearest note for tuning

PLAY NOTE=C4 or PLAY F=440

Adjustable concert pitch (SET A4=442, etc.)

Android App

Scans for ESP32-YinPitch-44k only (filtered)

Shows:

Current Note

Frequency

Cents offset

Confidence

Sends play/mute/start/stop commands

Accuracy Log Mode:

Collects cents error values

Computes average error, max error, stability percentage

Displays last raw JSON packet for debugging

Fully written in Kotlin + BLE GATT

🛠️ Hardware Setup (ESP32 + MAX9814)
Connections
MAX9814 Pin	ESP32 Pin
VDD	3.3V
GND	GND
OUT	GPIO 34 (ADC1_CH6)
Buzzer
Buzzer Pin	ESP32 Pin
+	GPIO 13
–	GND
🔧 Arduino Setup
Install Required Tools

Install ESP32 Arduino Core v2.0+
Boards Manager → Search “esp32” → Install

Required libraries:

NimBLE-Arduino


Set board in Arduino IDE:

Tools → Board → ESP32 Dev Module
Upload Speed: 921600 (optional)
Flash: 4 MB

📡 BLE Service Description

The ESP32 exposes a Nordic UART-style BLE service:

Purpose	UUID
Service	6E400001-B5A3-F393-E0A9-E50E24DCCA9E
RX (App → ESP)	6E400002-B5A3-F393-E0A9-E50E24DCCA9E
TX (ESP → App)	6E400003-B5A3-F393-E0A9-E50E24DCCA9E
💬 BLE Commands

Send these as plain text strings:

Command	Function
START	Begin pitch streaming
STOP	Pause pitch processing
MUTE	Silence buzzer
PLAY NOTE=C4	Plays musical note C4
PLAY F=440	Plays raw frequency
REF ON	Buzzer plays nearest note to your pitch
REF OFF	Stop reference tone
SET A4=442	Change tuning reference
PING	Returns JSON echo packet
📤 BLE JSON Output Format

ESP32 sends packets like:

{
  "f0": 293.66,
  "note": "D4",
  "cents": -12.5,
  "rms": 0.136,
  "prob": 0.82
}

Field	Description
f0	Frequency (after calibration)
note	Nearest musical note
cents	Sharp/flat offset
rms	Input signal energy
prob	YIN confidence metric
📱 Android App Setup
Requirements

Android Studio Flamingo or newer

Min SDK: 26

Permissions required:

BLUETOOTH_SCAN

BLUETOOTH_CONNECT

ACCESS_FINE_LOCATION

Build Instructions

Open folder in Android Studio

Build → Make Project

Run on a physical Android device

Click Scan → Pick ESP32-YinPitch-44k

📊 Accuracy Logging (App Feature)

Tapping “Start Accuracy Log”:

Clears historical pitch errors

Collects absolute cents deviations

When stopped:

Computes:

Avg cents error

Max deviation

Stability % = percent of samples within ±10 cents

Displays formatted results in the UI

🔌 Power Usage (ESP32)

Average measured draw:

150 mA @ 5V

≈0.75W continuous usage

Expected runtime:

Power Bank	Runtime
10,000 mAh	~39 hours
20,000 mAh	~79 hours
🔍 Troubleshooting
ESP32 Not Advertising

Ensure NimBLEDevice::startAdvertising() is called after disconnect

Check USB serial for:

[BLE] Advertising started

App Shows Write failed

Ensure ESP32 has:

NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR

Pitch Always Flat or Sharp

Adjust calibration:

#define CALIBRATION_SEMITONES  -3.7f


Use per-range correction if needed (B4 high offset, C2 low offset)

App Sees Too Many Devices

Check scanner filter:

if (!name.contains("ESP32") && !name.contains("YinPitch")) return

📄 License

MIT License — free to use, modify, and distribute.

🙌 Credits

Developed by Campbell Brown
Pitch engine based on the YIN algorithm, adapted for ESP32 real-time use.
BLE transport uses NimBLE-Arduino for stability and low RAM usage.