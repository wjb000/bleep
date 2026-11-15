// Minimal Feetech STS (SMS/STS protocol) driver over a UART + Waveshare Bus Servo Adapter (A) (half-duplex handled by the adapter).
#pragma once
#include <Arduino.h>

namespace sts {
static const uint8_t INST_READ = 0x02, INST_WRITE = 0x03, INST_SYNC_WRITE = 0x83;
static const uint8_t ADDR_TORQUE_ENABLE = 40, ADDR_ACC = 41, ADDR_GOAL_POS = 42, ADDR_PRESENT_POS = 56, ADDR_PRESENT_VOLT = 62, ADDR_PRESENT_TEMP = 63;

class Bus {
 public:
  HardwareSerial *s;
  void begin(HardwareSerial &ser, int rx, int tx, uint32_t baud) { s = &ser; s->begin(baud, SERIAL_8N1, rx, tx); s->setTimeout(3); }

  void write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n) {
    uint8_t len = n + 3, chk = id + len + INST_WRITE + addr;
    s->write(0xFF); s->write(0xFF); s->write(id); s->write(len); s->write(INST_WRITE); s->write(addr);
    for (uint8_t i = 0; i < n; i++) { s->write(data[i]); chk += data[i]; }
    s->write((uint8_t)~chk); s->flush(); drain(6);   // status packet
  }
  void torque(uint8_t id, bool on) { uint8_t v = on; write(id, ADDR_TORQUE_ENABLE, &v, 1); }

  // goal positions for many servos in one packet (no replies): pos (2) + time (2, 0) + speed (2, 0 = max)
  void syncGoal(const uint8_t *ids, const int16_t *pos, uint8_t n) {
    const uint8_t dl = 6; uint8_t len = (dl + 1) * n + 4, chk = 0xFE + len + INST_SYNC_WRITE + ADDR_GOAL_POS + dl;
    s->write(0xFF); s->write(0xFF); s->write(0xFE); s->write(len); s->write(INST_SYNC_WRITE); s->write(ADDR_GOAL_POS); s->write(dl);
    for (uint8_t i = 0; i < n; i++) {
      uint16_t p = (uint16_t)constrain(pos[i], 0, 4095);
      uint8_t b[7] = {ids[i], (uint8_t)(p & 0xFF), (uint8_t)(p >> 8), 0, 0, 0, 0};
      for (uint8_t k = 0; k < 7; k++) { s->write(b[k]); chk += b[k]; }
    }
    s->write((uint8_t)~chk); s->flush();
  }

  // read n bytes from addr; returns false on timeout / bad checksum
  bool read(uint8_t id, uint8_t addr, uint8_t *out, uint8_t n) {
    while (s->available()) s->read();
    uint8_t chk = id + 4 + INST_READ + addr + n;
    uint8_t pkt[8] = {0xFF, 0xFF, id, 4, INST_READ, addr, n, (uint8_t)~chk};
    s->write(pkt, 8); s->flush();
    uint8_t h[5];
    if (s->readBytes(h, 5) != 5 || h[0] != 0xFF || h[1] != 0xFF || h[2] != id) return false;
    uint8_t len = h[3];
    if (len != n + 2) return false;
    uint8_t buf[16];
    if (s->readBytes(buf, n + 1) != n + 1) return false;
    uint8_t c = id + len + h[4];
    for (uint8_t i = 0; i < n; i++) { out[i] = buf[i]; c += buf[i]; }
    return (uint8_t)~c == buf[n];
  }
  // present position (counts) and speed (counts/s, sign in bit 15)
  bool readPosSpeed(uint8_t id, int16_t &pos, int16_t &spd) {
    uint8_t b[4];
    if (!read(id, ADDR_PRESENT_POS, b, 4)) return false;
    pos = b[0] | (b[1] << 8);
    uint16_t v = b[2] | (b[3] << 8);
    spd = (v & 0x8000) ? -(int16_t)(v & 0x7FFF) : (int16_t)v;
    return true;
  }
  float readVoltage(uint8_t id) { uint8_t v; return read(id, ADDR_PRESENT_VOLT, &v, 1) ? v * 0.1f : -1.0f; }

 private:
  void drain(uint8_t n) { uint32_t t = micros(); uint8_t got = 0; while (got < n && micros() - t < 600) if (s->available()) { s->read(); got++; } }
};
}  // namespace sts
