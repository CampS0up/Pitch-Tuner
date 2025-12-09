// ================== INCLUDES ==================
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

// ================== MIC / ADC CONFIG ==================
#define MIC_ADC_GPIO       34          // MAX9814 OUT here (GPIO34 = ADC1_CH6)
#define ADC_CHANNEL        ADC1_CHANNEL_6

#define BUZZER_PIN         13

#define YIN_BUFFER_SIZE    2048        // samples per frame
#define YIN_THRESHOLD      0.15f       // YIN threshold

// Voice guard rails
#define MIN_F0_VOICE_HZ    55.0f       // below this: ignore
#define MAX_F0_VOICE_HZ    1100.0f     // above this: ignore

// ===== PITCH CALIBRATION =====
// You observed about 5 notes too high -> subtract 5 semitones.
#define CALIBRATION_SEMITONES   -5.0f
#define FREQ_CALIBRATION_FACTOR powf(2.0f, CALIBRATION_SEMITONES / 12.0f)

// ================== YIN STRUCT / FUNCTIONS ==================

typedef struct {
  int16_t bufferSize;
  int16_t halfBufferSize;
  float threshold;
  float probability;
  float *yinBuffer;
} Yin;

// static YIN buffer (avoid malloc for stability)
static float g_yinBuf[YIN_BUFFER_SIZE / 2];

// Forward declarations
void Yin_init(Yin *yin, int16_t bufferSize, float threshold);
void Yin_difference(Yin *yin, int16_t* buffer);
void Yin_cumulativeMeanNormalizedDifference(Yin *yin);
int16_t Yin_absoluteThreshold(Yin *yin);
float Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate);
float Yin_getPitch(Yin *yin, int16_t* buffer, float sampleRateHz);
float Yin_getProbability(Yin *yin);

// ----- YIN IMPLEMENTATION -----

void Yin_init(Yin *yin, int16_t bufferSize, float threshold) {
  yin->bufferSize = bufferSize;
  yin->halfBufferSize = bufferSize / 2;
  yin->probability = 0.0f;
  yin->threshold = threshold;

  yin->yinBuffer = g_yinBuf;
  for (int16_t i = 0; i < yin->halfBufferSize; i++) {
    yin->yinBuffer[i] = 0.0f;
  }
}

/**
 * Step 1: Difference function (autocorrelation-like)
 */
void Yin_difference(Yin *yin, int16_t* buffer) {
  int16_t tau, i;
  float delta;

  // Clear buffer each frame
  for (tau = 0; tau < yin->halfBufferSize; tau++) {
    yin->yinBuffer[tau] = 0.0f;
  }

  for (tau = 0; tau < yin->halfBufferSize; tau++) {
    for (i = 0; i < yin->halfBufferSize; i++) {
      delta = (float)buffer[i] - (float)buffer[i + tau];
      yin->yinBuffer[tau] += delta * delta;
    }
  }
}

/**
 * Step 2: Cumulative mean normalized difference
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin) {
  int16_t tau;
  float runningSum = 0.0f;
  yin->yinBuffer[0] = 1.0f;

  for (tau = 1; tau < yin->halfBufferSize; tau++) {
    runningSum += yin->yinBuffer[tau];
    if (runningSum == 0) {
      yin->yinBuffer[tau] = 1.0f;
    } else {
      yin->yinBuffer[tau] = yin->yinBuffer[tau] * tau / runningSum;
    }
  }
}

/**
 * Step 3: Absolute threshold
 */
int16_t Yin_absoluteThreshold(Yin *yin) {
  int16_t tau;

  for (tau = 2; tau < yin->halfBufferSize; tau++) {
    if (yin->yinBuffer[tau] < yin->threshold) {
      while (tau + 1 < yin->halfBufferSize &&
             yin->yinBuffer[tau + 1] < yin->yinBuffer[tau]) {
        tau++;
      }
      yin->probability = 1.0f - yin->yinBuffer[tau];
      break;
    }
  }

  if (tau == yin->halfBufferSize || yin->yinBuffer[tau] >= yin->threshold) {
    tau = -1;
    yin->probability = 0.0f;
  }

  return tau;
}

/**
 * Step 5: Parabolic interpolation
 */
float Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate) {
  int16_t x0, x2;
  float s0, s1, s2;
  float betterTau;

  if (tauEstimate < 1) {
    x0 = tauEstimate;
  } else {
    x0 = tauEstimate - 1;
  }

  if (tauEstimate + 1 < yin->halfBufferSize) {
    x2 = tauEstimate + 1;
  } else {
    x2 = tauEstimate;
  }

  if (x0 == tauEstimate) {
    if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x2]) {
      betterTau = (float)tauEstimate;
    } else {
      betterTau = (float)x2;
    }
  } else if (x2 == tauEstimate) {
    if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x0]) {
      betterTau = (float)tauEstimate;
    } else {
      betterTau = (float)x0;
    }
  } else {
    s0 = yin->yinBuffer[x0];
    s1 = yin->yinBuffer[tauEstimate];
    s2 = yin->yinBuffer[x2];
    betterTau = tauEstimate + (s2 - s0) / (2.0f * (2.0f * s1 - s2 - s0));
  }

  return betterTau;
}

/**
 * Pitch estimation given actual sampleRateHz
 */
float Yin_getPitch(Yin *yin, int16_t* buffer, float sampleRateHz) {
  int16_t tauEstimate = -1;
  float pitchInHertz = 0.0f;

  Yin_difference(yin, buffer);
  Yin_cumulativeMeanNormalizedDifference(yin);
  tauEstimate = Yin_absoluteThreshold(yin);

  if (tauEstimate != -1) {
    float betterTau = Yin_parabolicInterpolation(yin, tauEstimate);
    if (betterTau > 0.0f) {
      pitchInHertz = sampleRateHz / betterTau;
    }
  }

  return pitchInHertz;
}

float Yin_getProbability(Yin *yin) {
  return yin->probability;
}

// ================== NOTE MATH ==================

const char *NOTE_NAMES[12] = {
  "C", "C#", "D", "D#", "E", "F",
  "F#", "G", "G#", "A", "A#", "B"
};

void freqToNote(float f0, char *buf, size_t bufLen, float *centsOut) {
  if (f0 <= 0.0f) {
    strncpy(buf, "--", bufLen);
    buf[bufLen - 1] = '\0';
    if (centsOut) *centsOut = 0.0f;
    return;
  }

  float midi = 69.0f + 12.0f * log2f(f0 / 440.0f);
  int midiInt = (int)roundf(midi);
  int noteIndex = (midiInt % 12 + 12) % 12;
  int octave = midiInt / 12 - 1;

  float baseFreq = 440.0f * powf(2.0f, (midiInt - 69) / 12.0f);
  float cents = 1200.0f * log2f(f0 / baseFreq);

  snprintf(buf, bufLen, "%s%d", NOTE_NAMES[noteIndex], octave);
  if (centsOut) *centsOut = cents;
}

// ================== BUZZER / NOTE PLAY ==================

int noteNameToMidi(const char* s) {
  if (!s || !s[0]) return -1;

  char note = toupper((unsigned char)s[0]);
  int idx = -1;
  switch (note) {
    case 'C': idx = 0; break;
    case 'D': idx = 2; break;
    case 'E': idx = 4; break;
    case 'F': idx = 5; break;
    case 'G': idx = 7; break;
    case 'A': idx = 9; break;
    case 'B': idx = 11; break;
    default: return -1;
  }

  int i = 1;
  if (s[i] == '#' || s[i] == 'b' || s[i] == 'B') {
    if (s[i] == '#') idx += 1;
    else idx -= 1;
    i++;
  }

  if (!isdigit((unsigned char)s[i])) return -1;
  int octave = s[i] - '0';

  int midi = 12 * (octave + 1) + idx;  // C4=60, A4=69
  return midi;
}

float midiToFreq(int midi) {
  if (midi <= 0) return 0.0f;
  float semis = (float)(midi - 69);
  return 440.0f * powf(2.0f, semis / 12.0f);
}

void playNoteByName(const char* noteStr) {
  int midi = noteNameToMidi(noteStr);
  if (midi < 0) {
    Serial.print("[BUZZER] Bad note: ");
    Serial.println(noteStr);
    return;
  }
  float freq = midiToFreq(midi);
  Serial.print("[BUZZER] Play note "); Serial.print(noteStr);
  Serial.print(" (MIDI "); Serial.print(midi);
  Serial.print(") @ "); Serial.print(freq); Serial.println(" Hz");

  if (freq > 0.0f) {
    tone(BUZZER_PIN, (unsigned int)(freq + 0.5f)); // round to nearest Hz
  }
}

void muteBuzzer() {
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("[BUZZER] Muted");
}

// ================== BLE (NIMBLE) ==================

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pRxNotifyChar = nullptr;  // ESP32 -> phone
static NimBLECharacteristic* pTxWriteChar  = nullptr;  // phone -> ESP32
static bool deviceConnected = false;

#define UART_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_CHARACTERISTIC_RX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify
#define UART_CHARACTERISTIC_TX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Client disconnected, restarting advertising");
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (pAdv) {
      pAdv->start();
      Serial.println("[BLE] Advertising restarted");
    }
  }
};

class TxWriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    if (val.empty()) return;

    Serial.print("[BLE] RX command: ");
    Serial.println(val.c_str());

    // Expect "PLAY:NOTE" or "MUTE"
    if (val.rfind("PLAY:", 0) == 0) {
      const char* noteStr = val.c_str() + 5;
      String ns(noteStr);
      ns.trim();
      playNoteByName(ns.c_str());
    } else if (val.rfind("MUTE", 0) == 0) {
      muteBuzzer();
    }
  }
};

void setupBLE() {
  Serial.println("[BLE] Initializing NimBLE...");
  NimBLEDevice::init("ESP32-YinPitch");
  NimBLEDevice::setPower(ESP_PWR_LVL_P7); // optional: high power

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(UART_SERVICE_UUID);

  // RX: notify characteristic (ESP32 -> phone)
  pRxNotifyChar = pService->createCharacteristic(
    UART_CHARACTERISTIC_RX,
    NIMBLE_PROPERTY::NOTIFY
  );

  // TX: write characteristic (phone -> ESP32)
  pTxWriteChar = pService->createCharacteristic(
    UART_CHARACTERISTIC_TX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pTxWriteChar->setCallbacks(new TxWriteCallbacks());

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(UART_SERVICE_UUID);
  pAdv->setAppearance(0x0000);
  pAdv->start();                      // <-- IMPORTANT: start via pAdv

  Serial.println("[BLE] UART service started, advertising as ESP32-YinPitch");
}

// Send pitch line via BLE (if connected)
void sendPitchOverBle(const char* line) {
  if (!deviceConnected || !pRxNotifyChar) return;
  pRxNotifyChar->setValue((uint8_t*)line, strlen(line));
  pRxNotifyChar->notify();
}

// ================== GLOBALS FOR AUDIO ==================

Yin g_yin;
int16_t g_audioBuffer[YIN_BUFFER_SIZE];

// ================== PITCH PROCESSING ==================

void processAudioFrame() {
  // Sample 2048 points and measure actual Fs
  uint32_t t0 = micros();
  double sumSq = 0.0;
  const int32_t mid = 2048;  // 12-bit mid

  for (int i = 0; i < YIN_BUFFER_SIZE; i++) {
    int raw = analogRead(MIC_ADC_GPIO);  // 0..4095
    int32_t centered = raw - mid;
    if (centered > 32767) centered = 32767;
    if (centered < -32768) centered = -32768;
    g_audioBuffer[i] = (int16_t)centered;
    sumSq += (double)centered * (double)centered;
  }

  uint32_t t1 = micros();
  float frameSec = (t1 - t0) / 1000000.0f;
  if (frameSec <= 0.0f) frameSec = 1e-3f;
  float fs = (float)YIN_BUFFER_SIZE / frameSec;  // <-- real sampling rate

  float rms = sqrtf(sumSq / (double)YIN_BUFFER_SIZE) / 2048.0f; // normalized-ish

  // Run YIN
  float f0 = Yin_getPitch(&g_yin, g_audioBuffer, fs);
  float prob = Yin_getProbability(&g_yin);

  // Apply calibration to fix "5 notes too high"
  f0 *= FREQ_CALIBRATION_FACTOR;

  char noteName[8];
  float cents = 0.0f;

  // Guard rails
  if (f0 < MIN_F0_VOICE_HZ || f0 > MAX_F0_VOICE_HZ || prob < 0.3f || rms < 0.01f) {
    strcpy(noteName, "--");
    f0 = 0.0f;
    cents = 0.0f;
  } else {
    freqToNote(f0, noteName, sizeof(noteName), &cents);
  }

  // Print + BLE
  char line[160];
  snprintf(line, sizeof(line),
           "RMS=%.4f  f0=%.2f Hz  Note=%s  (%.1f cents)  conf=%.2f\n",
           rms, f0, noteName, cents, prob);
  Serial.print(line);
  sendPitchOverBle(line);
}

// ================== ARDUINO SETUP/LOOP ==================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] ESP32 YIN Pitch + BLE UART + Buzzer");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // ADC config
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_ADC_GPIO, ADC_11db); // 0-3.6V

  // Init YIN
  Yin_init(&g_yin, YIN_BUFFER_SIZE, YIN_THRESHOLD);

  // BLE
  setupBLE();
}

void loop() {
  processAudioFrame();
  delay(1); // small yield
}
