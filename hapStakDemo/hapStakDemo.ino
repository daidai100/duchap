// AtomS3向け hapStakDemo 移植版
// 元: ADACHACY hapStakDemo（https://github.com/bit-trade-one/ADACHACY-hapStak）

#include <M5Unified.h>
#include <math.h>

#include "FS.h"
#include "SPIFFS.h"

#include "AudioFileSourceSPIFFS.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorWAVRepeatable.h"
#include "AudioOutputI2S.h"

#define FILE_NAME "/file%02d.wav"
#define BOOT_NAME "/boot.wav"
#define DEFAULT_GAIN 1.0f
#define BEEP_GAIN 0.5f
#define ACCEL_THRESHOLD 3.0f
#define LOOP_DELAY 5

#define CONFIG_I2S_BCK_PIN      6
#define CONFIG_I2S_LRCK_PIN     8
#define CONFIG_I2S_DATA_PIN     5

AudioGeneratorWAVRepeatable *wav;
AudioFileSourceSPIFFS *file;
AudioOutputI2S *out;
AudioFileSourceID3 *id3;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 0);
  M5.Display.print("hapStak Demo Start\n");

  Serial.begin(115200);
  SPIFFS.begin();

  audioLogger = &Serial;
  out = new AudioOutputI2S();
  out->SetPinout(CONFIG_I2S_BCK_PIN, CONFIG_I2S_LRCK_PIN, CONFIG_I2S_DATA_PIN);
  out->SetGain(DEFAULT_GAIN);
  wav = new AudioGeneratorWAVRepeatable();

  if (M5.Imu.begin()) {
    M5.Display.println("IMU Init OK");
  } else {
    M5.Display.println("IMU Init FAILED");
  }

  playWAV(BOOT_NAME, BEEP_GAIN);
}

void loop() {
  static bool triggered = false;

  float ax, ay, az;
  M5.Imu.getAccel(&ax, &ay, &az);
  float a = sqrt(ax * ax + ay * ay + az * az);

  if (a > ACCEL_THRESHOLD && !triggered) {
    triggered = true;
    playWAV("/file01.wav", DEFAULT_GAIN);
  } else if (a < 1.5f) {
    triggered = false;
  }

  if (wav && wav->isRunning()) {
    if (!wav->loop()) {
      wav->stop();
    }
  }

  delay(LOOP_DELAY);
}

void playWAV(const char *filename, float gain) {
  if (wav->isRunning()) wav->stop();

  file = new AudioFileSourceSPIFFS(filename);
  id3 = new AudioFileSourceID3(file);

  out->SetGain(gain);
  wav->begin(id3, out);
}