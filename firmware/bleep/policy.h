// The walking policy: a 41 -> 128 -> 128 -> 10 ELU MLP exported from PPO (sim/export_policy.py). ~23k MACs, < 1 ms on the ESP32-S3.
#pragma once
#include <math.h>
#include "policy_weights.h"

static inline float elu(float x) { return x > 0.0f ? x : expm1f(x); }

// obs[POLICY_OBS] -> act[POLICY_ACT] in [-1, 1]
static void policy_forward(const float *obs, float *act) {
  static float h1[POLICY_H1], h2[POLICY_H2];
  for (int i = 0; i < POLICY_H1; i++) { float s = PB1[i]; for (int j = 0; j < POLICY_OBS; j++) s += PW1[i][j] * obs[j]; h1[i] = elu(s); }
  for (int i = 0; i < POLICY_H2; i++) { float s = PB2[i]; for (int j = 0; j < POLICY_H1; j++) s += PW2[i][j] * h1[j]; h2[i] = elu(s); }
  for (int i = 0; i < POLICY_ACT; i++) { float s = PB3[i]; for (int j = 0; j < POLICY_H2; j++) s += PW3[i][j] * h2[j]; act[i] = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s); }
}

// Observation layout = sim/env.py BleepEnv._obs (keep in sync):
// [0:3] projected gravity (body frame), [3:6] gyro*0.25, [6:9] cmd*CMD_SCALE, [9:19] q - Q_HOME, [19:29] qdot*DQ_SCALE,
// [29:39] last action, [39] sin(2*pi*phase), [40] cos(2*pi*phase)
static void build_obs(float *o, const float g[3], const float gyro[3], const float cmd[3], const float *q, const float *qd,
                      const float *last_act, float phase) {
  for (int i = 0; i < 3; i++) { o[i] = g[i]; o[3 + i] = gyro[i] * 0.25f; o[6 + i] = cmd[i] * CMD_SCALE[i]; }
  for (int i = 0; i < 10; i++) { o[9 + i] = q[i] - Q_HOME[i]; o[19 + i] = qd[i] * DQ_SCALE; o[29 + i] = last_act[i]; }
  o[39] = sinf(6.2831853f * phase); o[40] = cosf(6.2831853f * phase);
}
