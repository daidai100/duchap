#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>
#include <stdarg.h>
#include "driver/twai.h"
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"

// ==== I2S pins ====
#define PIN_DATA 5
#define PIN_BCLK 6
#define PIN_LRCK 8
#define AUDIO_I2S_PORT 0
#define AMP_CTRL_PIN (-1)

// ==== Tuning ====
static constexpr float DEFAULT_GAIN = 0.9f;
static constexpr int   IDLE_DELAY_MS = 2;

// ==== WAV ====
#include "file07.h"
#include "file08.h"
#include "file09.h"
#include "file10.h"
#include "file11.h"
#include "file12.h"
#include "file13.h"
#include "file14.h"

const char* patternNames[] = {
  "Pistol","Machine Gun","Bazooka","Tennis",
  "Wooden Bat","Iron Bat","Sword Slash","Sword Obj"
};
const unsigned char* wav_data[] = { file07, file08, file09, file10, file11, file12, file13, file14 };
const uint32_t wav_size[] = { sizeof(file07), sizeof(file08), sizeof(file09), sizeof(file10),
                              sizeof(file11), sizeof(file12), sizeof(file13), sizeof(file14) };

int  currentIndex = 0;

// ---------- UI ----------
void drawLine(int y, const char* fmt, ...) {
  char buf[128];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  M5.Display.setCursor(0, y);
  M5.Display.fillRect(0, y, 240, 12, BLACK);
  M5.Display.print(buf);
}
void showCurrent() {
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(1);
  drawLine(0, "[%s]", patternNames[currentIndex]);
  drawLine(12, "I2S D=%d B=%d L=%d", PIN_DATA, PIN_BCLK, PIN_LRCK);
  drawLine(24, "I2S port: %d", AUDIO_I2S_PORT);
  drawLine(36, "BtnA=Play, CAN=Play");
}

// ---------- Audio ----------
AudioGeneratorWAV*      g_wav  = nullptr;
AudioFileSourcePROGMEM* g_file = nullptr;
AudioOutputI2S*         g_out  = nullptr;

static inline bool isAudioRunning();
static inline void stopAndFreeAudio() {
  if (g_wav) { g_wav->stop(); delete g_wav; g_wav = nullptr; }
  if (g_file){ delete g_file;  g_file = nullptr; }
}
static inline bool isAudioRunning() {
  return g_wav && g_wav->isRunning();
}
bool beginPlay(const uint8_t* buf, size_t size) {
  stopAndFreeAudio();
  g_file = new AudioFileSourcePROGMEM(buf, size);
  g_wav  = new AudioGeneratorWAV();
  g_out->SetGain(DEFAULT_GAIN);
  g_out->SetChannels(1);
  if (g_wav->begin(g_file, g_out)) { drawLine(48, "AUDIO: begin OK"); return true; }
  drawLine(48, "AUDIO: begin FAILED");
  return false;
}
static uint8_t computeHapticIntensity() {
  return (isAudioRunning() ? 180 : 0);
}

// ---------- Buttons ----------
static constexpr int BTN_GPIO_1 = 7;
static constexpr int BTN_GPIO_2 = 38;
static constexpr int BTN_GPIO_3 = 39;

// BtnA ラッチ
static bool     btnA_latched = false;
static uint32_t btnA_latch_ts = 0;
static constexpr uint32_t BTN_LATCH_MS = 120;

void setupGpioButtons() {
  pinMode(BTN_GPIO_1, INPUT_PULLUP);
  pinMode(BTN_GPIO_2, INPUT_PULLUP);
  pinMode(BTN_GPIO_3, INPUT_PULLUP);
}
static inline uint8_t readButtonBits() {
  uint8_t b = 0;
  if (digitalRead(BTN_GPIO_1) == LOW) b |= (1 << 0); // GPIO7
  if (digitalRead(BTN_GPIO_2) == LOW) b |= (1 << 1); // GPIO38
  if (digitalRead(BTN_GPIO_3) == LOW) b |= (1 << 2); // GPIO39

  // BtnA: エッジ検出＋ラッチ
  if (M5.BtnA.wasPressed()) { btnA_latched = true; btnA_latch_ts = millis(); }
  if (btnA_latched) {
    b |= (1 << 4);
    if (millis() - btnA_latch_ts >= BTN_LATCH_MS) btnA_latched = false;
  }
  return b;
}

// ---------- CAN ----------
static constexpr gpio_num_t TWAI_TX = (gpio_num_t)2;  // G2
static constexpr gpio_num_t TWAI_RX = (gpio_num_t)1;  // G1
static constexpr uint32_t CAN_ID_TX = 0x120;          // Atom -> PC
static constexpr uint32_t CAN_ID_RX = 0x121;          // PC -> Atom

static constexpr uint32_t CAN_TX_PERIOD_MS = 50;
uint32_t last_can_tx_ms = 0;

bool initCAN() {
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_general_config_t gconf  = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = TWAI_TX,
    .rx_io = TWAI_RX,
    .clkout_io = (gpio_num_t)TWAI_IO_UNUSED,
    .bus_off_io = (gpio_num_t)TWAI_IO_UNUSED,
    .tx_queue_len = 8,
    .rx_queue_len = 8,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0
  };
  if (twai_driver_install(&gconf,&timing,&filter)!=ESP_OK) return false;
  if (twai_start()!=ESP_OK) return false;
  return true;
}
void canPeriodicSend() {
  uint32_t now=millis();
  if (now-last_can_tx_ms<CAN_TX_PERIOD_MS) return;
  last_can_tx_ms=now;

  uint8_t intensity = computeHapticIntensity();
  uint8_t btn_bits  = readButtonBits();

  twai_message_t msg={};
  msg.identifier=CAN_ID_TX;
  msg.extd=0; msg.rtr=0;
  msg.data_length_code=2;     // DLC=2 (intensity + buttons)
  msg.data[0]=intensity;
  msg.data[1]=btn_bits;
  twai_transmit(&msg, pdMS_TO_TICKS(5));
}
void canPollReceiveAndApply() {
  twai_message_t rx;
  while (twai_receive(&rx,0)==ESP_OK) {
    if (rx.identifier==CAN_ID_RX && rx.data_length_code>=1) {
      uint8_t v=rx.data[0];
      int  preset  = v & 0x07;
      bool trigger = (v & 0x80)!=0;
      if (preset!=currentIndex) { currentIndex = preset % 8; showCurrent(); }
      if (trigger) { beginPlay(wav_data[currentIndex], wav_size[currentIndex]); }
    }
  }
}

// ---------- setup/loop ----------
void setup() {
  auto cfg=M5.config();
  M5.begin(cfg);
  M5.Speaker.end();
  if (AMP_CTRL_PIN >= 0) { pinMode(AMP_CTRL_PIN, OUTPUT); digitalWrite(AMP_CTRL_PIN, HIGH); }

  M5.Display.setFont(&fonts::FreeMonoBold9pt7b);
  M5.Display.setTextDatum(top_left);
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setRotation(1);  // 90度回転

  Serial.begin(115200);
  audioLogger = &Serial;

#if AUDIO_I2S_PORT == 0
  g_out = new AudioOutputI2S(I2S_NUM_0);
#else
  g_out = new AudioOutputI2S(I2S_NUM_1);
#endif
  g_out->SetPinout(PIN_BCLK, PIN_LRCK, PIN_DATA);
  g_out->SetGain(DEFAULT_GAIN);
  g_out->SetChannels(1);

  setupGpioButtons();
  bool can_ok = initCAN();

  drawLine(60, "CAN: %s", can_ok ? "OK" : "NG");
  showCurrent();
}

void loop() {
  M5.update();                 // ← BtnA.wasPressed() のため必須

  if (g_wav && g_wav->isRunning()) {
    if (!g_wav->loop()) { stopAndFreeAudio(); drawLine(72, "AUDIO: finished"); }
  }
  if (M5.BtnA.wasPressed()) beginPlay(wav_data[currentIndex], wav_size[currentIndex]);

  canPeriodicSend();           // ← M5.update() 後に呼ぶ
  canPollReceiveAndApply();

  delay(IDLE_DELAY_MS);
}
