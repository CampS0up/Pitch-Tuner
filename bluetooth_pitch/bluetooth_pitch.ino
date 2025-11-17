/*
  ESP32 + MAX9814 + YIN Pitch + I2S ADC @ 44.1 kHz + BLE + Buzzer

  Hardware:
    - ESP32 DevKit (classic ESP32)
    - MAX9814 Mic:
        VDD -> 3V3
        GND -> GND
        OUT -> GPIO34  (ADC1_CH6, used via I2S ADC)
    - Piezo buzzer:
        +  -> GPIO13
        -  -> GND
*/

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <NimBLEDevice.h>
#include "driver/i2s.h"
#include "driver/adc.h"

// =================== CONFIG ===================
#define MIC_ADC_GPIO       34          // MAX9814 OUT here
#define ADC_CHANNEL        ADC1_CHANNEL_6  // GPIO34 = ADC1_CH6

#define BUZZER_PIN         13

#define YIN_SAMPLING_RATE  35000       // Hz
#define YIN_BUFFER_SIZE    2048        // samples per frame
#define YIN_THRESHOLD      0.15f       // YIN threshold

// Voice guard rails
#define MIN_F0_VOICE_HZ    55.0f       // below this: ignore
#define MAX_F0_VOICE_HZ    1100.0f      // above this: fold down

// RMS threshold to ignore very quiet frames
#define MIN_RMS_FOR_VOICE  0.004f

// BLE UUIDs (Nordic UART style)
static NimBLEUUID UUID_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID UUID_RX     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID UUID_TX     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

#define REPORT_INTERVAL_MS  60

// =================== YIN STRUCT ===================
typedef struct {
  int16_t bufferSize;
  int16_t halfBufferSize;
  float   threshold;
  float   probability;
  float*  yinBuffer;
} Yin;

// Static storage for YIN buffer to avoid malloc
static float yinBufferData[YIN_BUFFER_SIZE / 2];
static Yin   yin;

// =================== GLOBALS ===================
static int16_t sampleBuffer[YIN_BUFFER_SIZE];
static float   fHist[5] = {0,0,0,0,0};
static int     fIdx     = 0;

static bool    g_running    = true;
static bool    g_refToneOn  = false;
static float   g_A4         = 440.0f;  // reference A4

static NimBLEServer*         g_server   = nullptr;
static NimBLECharacteristic* g_txChar   = nullptr;
static NimBLECharacteristic* g_rxChar   = nullptr;
static bool                  g_connected = false;
static uint32_t              g_lastReport = 0;
static uint32_t              g_lastRefUpdate = 0;

// =================== YIN FUNCTIONS (DingKe-based) ===================

void Yin_difference(Yin* yin, int16_t* buffer) {
  int16_t i;
  int16_t tau;
  float   delta;

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

void Yin_cumulativeMeanNormalizedDifference(Yin* yin) {
  int16_t tau;
  float runningSum = 0.0f;

  yin->yinBuffer[0] = 1.0f;

  for (tau = 1; tau < yin->halfBufferSize; tau++) {
    runningSum += yin->yinBuffer[tau];
    if (runningSum > 0.0f) {
      yin->yinBuffer[tau] *= (float)tau / runningSum;
    } else {
      yin->yinBuffer[tau] = 1.0f;
    }
  }
}

int16_t Yin_absoluteThreshold(Yin* yin) {
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

float Yin_parabolicInterpolation(Yin* yin, int16_t tauEstimate) {
  float   betterTau;
  int16_t x0;
  int16_t x2;

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
    float s0 = yin->yinBuffer[x0];
    float s1 = yin->yinBuffer[tauEstimate];
    float s2 = yin->yinBuffer[x2];
    betterTau = tauEstimate + (s2 - s0) / (2.0f * (2.0f * s1 - s2 - s0));
  }

  return betterTau;
}

void Yin_init(Yin* y, int16_t bufferSize, float threshold) {
  y->bufferSize     = bufferSize;
  y->halfBufferSize = bufferSize / 2;
  y->probability    = 0.0f;
  y->threshold      = threshold;
  y->yinBuffer      = yinBufferData;

  for (int16_t i = 0; i < y->halfBufferSize; i++) {
    y->yinBuffer[i] = 0.0f;
  }
}

float Yin_getPitch(Yin* y, int16_t* buffer) {
  int16_t tauEstimate = -1;
  float   pitchInHz   = -1.0f;

  Yin_difference(y, buffer);
  Yin_cumulativeMeanNormalizedDifference(y);
  tauEstimate = Yin_absoluteThreshold(y);

  if (tauEstimate != -1) {
    float tauRefined = Yin_parabolicInterpolation(y, tauEstimate);
    if (tauRefined > 0.0f) {
      pitchInHz = (float)YIN_SAMPLING_RATE / tauRefined;
    }
  }

  return pitchInHz;
}

float Yin_getProbability(Yin* y) {
  return y->probability;
}

// =================== HELPER: RMS & NOTE MAPPING ===================

float computeRMS_int16(const int16_t* buf, int N) {
  double acc = 0.0;
  for (int i = 0; i < N; i++) {
    acc += (double)buf[i] * (double)buf[i];
  }
  // 12-bit data scaled into int16; normalization is approximate
  return sqrtf((float)(acc / N)) / 4096.0f;
}

void hzToNote(float f, char* outName, float* outCents, float A4 = 440.0f) {
  static const char* NAMES[12] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
  };

  if (f <= 0.0f) {
    strcpy(outName, "--");
    *outCents = 0.0f;
    return;
  }

  float midi = 69.0f + 12.0f * (logf(f / A4) / logf(2.0f));
  int   r    = (int)floorf(midi + 0.5f);
  int   pc   = (r % 12 + 12) % 12;
  int   oct  = r / 12 - 1;

  sprintf(outName, "%s%d", NAMES[pc], oct);
  float cents = (midi - (float)r) * 100.0f;
  *outCents = cents;
}

// nearest equal-tempered note freq
float nearestNoteFreq(float f, float A4 = 440.0f) {
  if (f <= 0.0f) return 0.0f;
  float midi = 69.0f + 12.0f * (logf(f / A4) / logf(2.0f));
  int   r    = (int)floorf(midi + 0.5f);
  return A4 * powf(2.0f, (float)(r - 69) / 12.0f);
}

// parse "C4", "G#3", etc.
float noteNameToFreq(const String& s, float A4 = 440.0f) {
  if (s.length() < 2) return 0.0f;
  char n0 = toupper(s[0]);
  char n1 = (s[1] == '#' || s[1] == 'b') ? s[1] : '\0';

  int idx = 1;
  if (n1 == '#' || n1 == 'b') idx = 2;

  int octave = s.substring(idx).toInt();
  int pc = -1;

  switch (n0) {
    case 'C': pc = 0; break;
    case 'D': pc = 2; break;
    case 'E': pc = 4; break;
    case 'F': pc = 5; break;
    case 'G': pc = 7; break;
    case 'A': pc = 9; break;
    case 'B': pc = 11; break;
    default: return 0.0f;
  }
  if (n1 == '#') pc += 1;
  if (n1 == 'b') pc -= 1;
  pc = (pc % 12 + 12) % 12;

  int midi = (octave + 1) * 12 + pc;
  return A4 * powf(2.0f, (float)(midi - 69) / 12.0f);
}

// =================== I2S ADC CAPTURE ===================

void setupI2SADC() {
  // Configure ADC1 channel
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

  // Set up I2S in ADC mode
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = YIN_SAMPLING_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,  // data in one channel is fine
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_NUM_0);
}

// Read 2048 samples via I2S ADC into sampleBuffer
void captureFrame2048() {
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, (void*)sampleBuffer, sizeof(sampleBuffer),
           &bytes_read, portMAX_DELAY);

  int count = bytes_read / sizeof(int16_t);
  if (count > YIN_BUFFER_SIZE) count = YIN_BUFFER_SIZE;

  // I2S ADC returns 12-bit data left-aligned in 16-bit
  for (int i = 0; i < count; i++) {
    int16_t raw = sampleBuffer[i];
    sampleBuffer[i] = raw >> 4;  // scale to 0..4095-ish
  }
}

// =================== BUZZER ===================

void buzzerPlayFreq(float f) {
  if (f <= 0.0f) {
    noTone(BUZZER_PIN);
  } else {
    unsigned freq = (unsigned)roundf(f);
    if (freq < 20) freq = 20;
    tone(BUZZER_PIN, freq);
  }
}

void buzzerMute() {
  noTone(BUZZER_PIN);
}

// =================== COMMAND HANDLING / BLE ===================

void handleCommand(String cmd); // forward

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    handleCommand(String(v.c_str()));
  }
};

class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_connected = true;
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    g_connected = false;
    Serial.println("[BLE] Disconnected, restarting advertising");
    NimBLEDevice::startAdvertising();
  }
};

void setupBLE() {
  NimBLEDevice::init("ESP32-YinPitch-44k");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEDevice::deleteAllBonds();
  NimBLEDevice::setSecurityAuth(false, false, false);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new SrvCB());

  NimBLEService* svc = g_server->createService(UUID_SERVICE);
  g_txChar = svc->createCharacteristic(UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  g_rxChar = svc->createCharacteristic(UUID_RX,
                                       NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  g_rxChar->setCallbacks(new RxCB());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->stop();
  adv->addServiceUUID(UUID_SERVICE);

  NimBLEAdvertisementData advData;
  advData.setName("ESP32-YinPitch-44k");
  adv->setAdvertisementData(advData);

  bool ok = adv->start();
  Serial.printf("[BLE] Advertising %s\n", ok ? "started" : "FAILED");
}

void ensureAdvertising() {
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
      Serial.println("[BLE] Adv stopped, restarting");
      NimBLEDevice::startAdvertising();
    }
  }
}

String serialBuf;

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  Serial.printf("[CMD] %s\n", cmd.c_str());

  if (cmd.equalsIgnoreCase("START")) {
    g_running = true;
  } else if (cmd.equalsIgnoreCase("STOP")) {
    g_running = false;
    buzzerMute();
  } else if (cmd.equalsIgnoreCase("REF ON")) {
    g_refToneOn = true;
  } else if (cmd.equalsIgnoreCase("REF OFF")) {
    g_refToneOn = false;
    buzzerMute();
  } else if (cmd.startsWith("SET A4=")) {
    float f = cmd.substring(7).toFloat();
    if (f >= 400.0f && f <= 480.0f) {
      g_A4 = f;
      Serial.printf("A4 set to %.2f Hz\n", g_A4);
    }
  } else if (cmd.equalsIgnoreCase("PING")) {
    char msg[64];
    int n = snprintf(msg, sizeof(msg),
                     "{\"echo\":\"PONG\",\"ms\":%lu}", (unsigned long)millis());
    Serial.println(msg);
    if (g_txChar) {
      g_txChar->setValue((uint8_t*)msg, n);
      g_txChar->notify();
    }
  } else if (cmd.equalsIgnoreCase("MUTE")) {
    buzzerMute();
  } else if (cmd.startsWith("PLAY F=")) {
    float f = cmd.substring(7).toFloat();
    buzzerPlayFreq(f);
  } else if (cmd.startsWith("PLAY NOTE=")) {
    String noteStr = cmd.substring(10);
    noteStr.trim();
    float f = noteNameToFreq(noteStr, g_A4);
    buzzerPlayFreq(f);
  }
}

// =================== ARDUINO SETUP/LOOP ===================

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerMute();

  Yin_init(&yin, YIN_BUFFER_SIZE, YIN_THRESHOLD);
  setupI2SADC();
  setupBLE();

  Serial.println("ESP32 + MAX9814 + YIN @ 44.1 kHz + BLE + Buzzer ready.");
  Serial.println("Mic OUT -> GPIO34, Buzzer -> GPIO13.");
}

void loop() {
  // Commands from Serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(serialBuf);
      serialBuf = "";
    } else {
      serialBuf += c;
      if (serialBuf.length() > 200) serialBuf.remove(0);
    }
  }

  // 1) Capture audio
  captureFrame2048();

  // 2) Compute RMS
  float rms = computeRMS_int16(sampleBuffer, YIN_BUFFER_SIZE);

  // 3) Run YIN
  float pitch = 0.0f;
  float prob  = 0.0f;

  if (g_running && rms >= MIN_RMS_FOR_VOICE) {
    float f0 = Yin_getPitch(&yin, sampleBuffer);
    prob     = Yin_getProbability(&yin);

    if (prob < 0.1f || f0 <= 0.0f) {
      pitch = 0.0f;
    } else {
      // Fold down harmonics into voice range
      while (f0 > MAX_F0_VOICE_HZ) {
        f0 *= 0.5f;
      }
      if (f0 < MIN_F0_VOICE_HZ) {
        pitch = 0.0f;
      } else {
        pitch = f0;
      }
    }
  } else {
    pitch = 0.0f;
    prob  = 0.0f;
  }

  // 4) Median smoothing (last 5 frames)
  fHist[fIdx] = pitch;
  fIdx = (fIdx + 1) % 5;

  float tmp[5]; int n = 0;
  for (int i = 0; i < 5; i++) {
    if (fHist[i] > 0.0f) tmp[n++] = fHist[i];
  }
  float pitchMed = 0.0f;
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (tmp[j] < tmp[i]) {
          float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t;
        }
      }
    }
    pitchMed = tmp[n / 2];
  }

  // 5) Note + cents
  char note[8];
  float cents = 0.0f;
  hzToNote(pitchMed, note, &cents, g_A4);

  // Serial debug
  Serial.printf("RMS=%.4f  f0=%.2f Hz  Note=%s  (%.1f cents)  prob=%.2f\n",
                rms, pitchMed, note, cents, prob);

  // BLE JSON
  uint32_t now = millis();
  if (g_connected && (now - g_lastReport) >= REPORT_INTERVAL_MS) {
    if (g_txChar) {
      char payload[200];
      int nbytes = snprintf(payload, sizeof(payload),
                            "{\"f0\":%.2f,\"note\":\"%s\",\"cents\":%.1f,"
                            "\"rms\":%.3f,\"prob\":%.2f}",
                            pitchMed, note, cents, rms, prob);
      g_txChar->setValue((uint8_t*)payload, nbytes);
      g_txChar->notify();
    }
    g_lastReport = now;
  }

  // Reference tone on buzzer
  if (g_refToneOn) {
    if (pitchMed > 0.0f) {
      if (now - g_lastRefUpdate > 80) {
        float refF = nearestNoteFreq(pitchMed, g_A4);
        buzzerPlayFreq(refF);
        g_lastRefUpdate = now;
      }
    } else {
      buzzerMute();
    }
  }

  ensureAdvertising();
}
