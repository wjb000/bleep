// MPU-6050 over I2C (no library) + a complementary filter that tracks the gravity direction in the body frame.
// Mount: chip x -> robot forward, y -> left, z -> up (see docs/ASSEMBLY.md). Change AXIS_MAP if mounted differently.
#pragma once
#include <Wire.h>
#include <math.h>

struct Imu {
  float g[3] = {0, 0, -1};     // unit gravity in body frame (what the policy calls projected gravity)
  float gyro[3] = {0, 0, 0};   // rad/s
  float bias[3] = {0, 0, 0};
  bool ok = false;
  static const uint8_t ADDR = 0x68;
  void wr(uint8_t r, uint8_t v) { Wire.beginTransmission(ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
  bool begin() {
    Wire.beginTransmission(ADDR); ok = Wire.endTransmission() == 0;
    if (!ok) return false;
    wr(0x6B, 0x01);  // wake, PLL clock
    wr(0x1A, 0x03);  // DLPF 44 Hz
    wr(0x1B, 0x08);  // gyro +-500 dps
    wr(0x1C, 0x08);  // accel +-4 g
    delay(50);
    float s[3] = {0, 0, 0}; float a[3], w[3];
    for (int i = 0; i < 200; i++) { raw(a, w); for (int k = 0; k < 3; k++) s[k] += w[k]; delay(2); }   // robot must be still at boot
    for (int k = 0; k < 3; k++) bias[k] = s[k] / 200;
    return true;
  }
  void raw(float a[3], float w[3]) {
    Wire.beginTransmission(ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR, 14);
    int16_t v[7]; for (int i = 0; i < 7; i++) v[i] = (Wire.read() << 8) | Wire.read();
    for (int k = 0; k < 3; k++) { a[k] = v[k] / 8192.0f; w[k] = v[4 + k] / 65.5f * 0.0174533f; }
  }
  void update(float dt) {
    float a[3], w[3]; raw(a, w);
    for (int k = 0; k < 3; k++) gyro[k] = w[k] - bias[k];
    // rotate the gravity estimate by -omega*dt (body frame), then pull it toward the accelerometer (which reads -g)
    float gx = g[0] + dt * (gyro[2] * g[1] - gyro[1] * g[2]);
    float gy = g[1] + dt * (gyro[0] * g[2] - gyro[2] * g[0]);
    float gz = g[2] + dt * (gyro[1] * g[0] - gyro[0] * g[1]);
    float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (an > 0.6f && an < 1.4f) { const float k = 0.02f; gx += k * (-a[0] / an - gx); gy += k * (-a[1] / an - gy); gz += k * (-a[2] / an - gz); }
    float n = sqrtf(gx * gx + gy * gy + gz * gz); g[0] = gx / n; g[1] = gy / n; g[2] = gz / n;
  }
  float tilt_deg() const { return acosf(fminf(1.0f, fmaxf(-1.0f, -g[2]))) * 57.2958f; }
};
