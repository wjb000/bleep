// Droid chirps on a passive piezo (LEDC PWM). Non-blocking: queue a pattern, call update() every loop.
#pragma once
#include <Arduino.h>
#include "config.h"
struct Chirp { uint16_t f0, f1, ms; };
struct Sounds {
  Chirp q[8]; int n = 0, i = 0; uint32_t t0 = 0; bool on = false;
  void begin() { ledcAttach(PIN_BUZZER, 2000, 8); ledcWrite(PIN_BUZZER, 0); }
  void play(const Chirp *c, int k) { n = min(k, 8); for (int j = 0; j < n; j++) q[j] = c[j]; i = 0; t0 = millis(); on = true; }
  void happy() { static const Chirp c[] = {{1800, 2600, 70}, {2600, 3400, 60}, {3000, 3800, 90}}; play(c, 3); }
  void curious() { static const Chirp c[] = {{1200, 2400, 160}}; play(c, 1); }
  void sad() { static const Chirp c[] = {{2200, 900, 300}}; play(c, 1); }
  void update() {
    if (!on) return;
    uint32_t dt = millis() - t0;
    if (dt >= q[i].ms) { i++; t0 = millis(); dt = 0; if (i >= n) { on = false; ledcWrite(PIN_BUZZER, 0); return; } }
    float u = (float)dt / q[i].ms; ledcWriteTone(PIN_BUZZER, q[i].f0 + (q[i].f1 - q[i].f0) * u); ledcWrite(PIN_BUZZER, 96);
  }
};
