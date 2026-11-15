// Expressive eye on the 1.28" round GC9A01 (Arduino_GFX library). One big droid pupil that looks, blinks and changes mood.
#pragma once
#include <Arduino_GFX_Library.h>
#include "config.h"

enum Mood { MOOD_IDLE, MOOD_HAPPY, MOOD_CURIOUS, MOOD_SLEEPY, MOOD_SAD, MOOD_LOWBATT };

struct Face {
  Arduino_DataBus *bus = nullptr; Arduino_GFX *gfx = nullptr;
  float look_x = 0, look_y = 0;     // -1..1
  Mood mood = MOOD_IDLE; uint32_t next_blink = 0, blink_until = 0;
  uint16_t accent = 0x2D7F;         // teal
  void begin() {
    bus = new Arduino_ESP32SPI(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI, GFX_NOT_DEFINED);
    gfx = new Arduino_GC9A01(bus, PIN_LCD_RST, 0, true);
    gfx->begin(40000000); gfx->fillScreen(0x0000);
  }
  void draw() {
    uint32_t now = millis();
    if (now > next_blink) { blink_until = now + 120; next_blink = now + 2500 + random(3000); }
    bool blink = now < blink_until || mood == MOOD_SLEEPY;
    gfx->startWrite();
    gfx->fillScreen(0x0000);
    int cx = 120 + (int)(look_x * 38), cy = 120 + (int)(look_y * 30);
    uint16_t ring = mood == MOOD_SAD ? 0x435C : mood == MOOD_LOWBATT ? 0xF800 : mood == MOOD_HAPPY ? 0xFD20 : accent;
    gfx->fillCircle(120, 120, 112, 0x10A2);                         // iris backdrop
    if (blink) { gfx->fillRoundRect(cx - 60, cy - 6, 120, 12, 6, ring); }
    else {
      int r = mood == MOOD_CURIOUS ? 62 : mood == MOOD_SAD ? 44 : 54;
      gfx->fillCircle(cx, cy, r, ring); gfx->fillCircle(cx, cy, r - 14, 0x0000);
      gfx->fillCircle(cx - r / 3, cy - r / 3, 9, 0xFFFF);           // glint
      if (mood == MOOD_HAPPY) gfx->fillRect(cx - r, cy + r / 3, 2 * r, r, 0x10A2);   // smiling lower lid
      if (mood == MOOD_SAD) gfx->fillTriangle(cx - r, cy - r, cx + r, cy - r, cx - r, cy, 0x10A2);
    }
    gfx->endWrite();
  }
};
