// Microphone loudness (PDM mic on the XIAO Sense) and camera motion tracking (OV3660/OV5640), run on core 0.
#pragma once
#include <Arduino.h>
#include <ESP_I2S.h>
#include "esp_camera.h"
#include "config.h"

struct Senses {
  volatile float loud = 0;          // short-term mic RMS (0..1)
  volatile float motion_x = 0;      // -1 (left of image) .. 1 (right), centroid of frame-to-frame change
  volatile float motion_amt = 0;    // 0..1
  bool cam_ok = false, mic_ok = false;
  I2SClass i2s;
  uint8_t *prev = nullptr;

  bool beginCamera() {
    camera_config_t c = {};
    c.pin_pwdn = -1; c.pin_reset = -1; c.pin_xclk = 10; c.pin_sccb_sda = 40; c.pin_sccb_scl = 39;
    c.pin_d7 = 48; c.pin_d6 = 11; c.pin_d5 = 12; c.pin_d4 = 14; c.pin_d3 = 16; c.pin_d2 = 18; c.pin_d1 = 17; c.pin_d0 = 15;
    c.pin_vsync = 38; c.pin_href = 47; c.pin_pclk = 13;           // XIAO ESP32S3 Sense camera map (Seeed CameraWebServer)
    c.xclk_freq_hz = 20000000; c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_GRAYSCALE; c.frame_size = FRAMESIZE_QQVGA; c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
    cam_ok = esp_camera_init(&c) == ESP_OK;
    if (cam_ok) prev = (uint8_t *)ps_malloc(160 * 120);
    return cam_ok;
  }
  bool beginMic() {
    i2s.setPinsPdmRx(PIN_MIC_CLK, PIN_MIC_DATA);
    mic_ok = i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    return mic_ok;
  }
  void pollMic() {
    if (!mic_ok) return;
    int16_t buf[256]; size_t n = i2s.readBytes((char *)buf, sizeof(buf)) / 2;
    if (!n) return;
    float s = 0; for (size_t i = 0; i < n; i++) s += (float)buf[i] * buf[i];
    float rms = sqrtf(s / n) / 32768.0f;
    loud = 0.8f * loud + 0.2f * rms;
  }
  void pollCamera() {
    if (!cam_ok) return;
    camera_fb_t *fb = esp_camera_fb_get(); if (!fb) return;
    const uint8_t *p = fb->buf; long sx = 0, cnt = 0;
    for (int y = 0; y < 120; y += 2) for (int x = 0; x < 160; x += 2) {
      int i = y * 160 + x; int dlt = abs((int)p[i] - (int)prev[i]);
      if (dlt > 28) { sx += x; cnt++; }
      prev[i] = p[i];
    }
    esp_camera_fb_return(fb);
    float amt = cnt / 4800.0f;
    motion_amt = 0.7f * motion_amt + 0.3f * amt;
    if (cnt > 40) motion_x = 0.7f * motion_x + 0.3f * ((float)sx / cnt / 80.0f - 1.0f);
  }
};
