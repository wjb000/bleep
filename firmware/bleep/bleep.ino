// Bleep firmware (Arduino core for ESP32 3.x, board: "XIAO_ESP32S3", PSRAM: OPI).
// Core 1: 50 Hz control loop (IMU -> observation -> policy -> servo targets). Core 0: face, sounds, mic, camera, Wi-Fi remote.
// Libraries: GFX Library for Arduino (Arduino_GFX). Everything else is in the ESP32 core.
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "sts_bus.h"
#include "imu.h"
#include "policy.h"
#include "face.h"
#include "sounds.h"
#include "senses.h"

sts::Bus bus; Imu imu; Face face; Sounds snd; Senses sen; WebServer web(80);
enum State { ST_BOOT, ST_STAND, ST_WALK, ST_FALLEN, ST_SIT };
volatile State state = ST_BOOT;
volatile float cmd[3] = {0, 0, 0};          // vx (m/s), vy (m/s), yaw rate (rad/s) from the remote
float q[N_JOINTS], qd[N_JOINTS], target[N_JOINTS], last_act[10] = {0}, phase = 0, neck_target = 0;
float batt_v = 6.4f; uint32_t t_state = 0;

static inline float count2rad(int j, int16_t c) { return JOINT_SIGN[j] * (c - JOINT_ZERO[j]) * (6.2831853f / 4096.0f); }
static inline int16_t rad2count(int j, float r) { return JOINT_ZERO[j] + (int16_t)lroundf(JOINT_SIGN[j] * r * (4096.0f / 6.2831853f)); }

void readJoints() {
  for (int j = 0; j < N_JOINTS; j++) {
    int16_t p, s;
    if (bus.readPosSpeed(SERVO_ID[j], p, s)) { q[j] = count2rad(j, p); qd[j] = JOINT_SIGN[j] * s * (6.2831853f / 4096.0f); }
  }
}
void sendTargets() {
  int16_t c[N_JOINTS];
  for (int j = 0; j < N_JOINTS; j++) c[j] = rad2count(j, constrain(target[j], Q_MIN[j], Q_MAX[j]));
  bus.syncGoal(SERVO_ID, c, N_JOINTS);
}
void setState(State s) { state = s; t_state = millis(); }

// ---------------------------------------------------------------- core 0: personality + remote
const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta name=viewport content="width=device-width"><title>Bleep</title>
<style>body{font:18px system-ui;background:#111;color:#eee;text-align:center}button{width:84px;height:64px;margin:6px;font-size:22px;border-radius:14px;border:0;background:#2b8;color:#000}</style>
<h2>Bleep</h2><div><button onclick="c(0,0,0.8)">&#8630;</button><button onclick="c(0.15,0,0)">&#9650;</button><button onclick="c(0,0,-0.8)">&#8631;</button></div>
<div><button onclick="c(0,0.06,0)">&#9664;</button><button onclick="c(0,0,0)">&#9632;</button><button onclick="c(0,-0.06,0)">&#9654;</button></div>
<div><button onclick="c(-0.08,0,0)">&#9660;</button><button onclick="fetch('/mode?m=walk')">walk</button><button onclick="fetch('/mode?m=stand')">stand</button></div>
<script>function c(x,y,w){fetch(`/cmd?vx=${x}&vy=${y}&wz=${w}`)}</script>)HTML";

void taskSenses(void *) {
  uint32_t last_face = 0; bool touched = false; uint32_t touch_t = 0; uint16_t touch_base = touchRead(PIN_TOUCH);
  for (;;) {
    sen.pollMic(); sen.pollCamera(); snd.update(); web.handleClient();
    // touch: pat the dome -> happy + toggle walk / stand
    uint16_t tv = touchRead(PIN_TOUCH);
    bool now = tv > touch_base * 1.15f;
    if (now && !touched) { touch_t = millis(); snd.happy(); face.mood = MOOD_HAPPY; if (state == ST_STAND) setState(ST_WALK); else if (state == ST_WALK) setState(ST_STAND); }
    touched = now;
    // sound: a loud noise makes it curious and look around
    if (sen.loud > 0.08f && face.mood != MOOD_CURIOUS) { face.mood = MOOD_CURIOUS; snd.curious(); }
    // camera: the dome follows motion (proportional, slow)
    if (sen.motion_amt > 0.01f) neck_target = constrain(neck_target - 0.04f * sen.motion_x, -1.5f, 1.5f);
    face.look_x = constrain(-sen.motion_x, -1.0f, 1.0f) * (sen.motion_amt > 0.01f);
    if (batt_v < LOW_BATT_V) face.mood = MOOD_LOWBATT;
    if (millis() - touch_t > 3000 && face.mood != MOOD_LOWBATT && state != ST_FALLEN && sen.loud < 0.05f) face.mood = MOOD_IDLE;
    if (millis() - last_face > 50) { face.draw(); last_face = millis(); }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  bus.begin(Serial1, PIN_BUS_RX, PIN_BUS_TX, BUS_BAUD);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  face.begin(); snd.begin();
  if (!imu.begin()) Serial.println("IMU not found");
  sen.beginCamera(); sen.beginMic();
  WiFi.softAP("Bleep-" + String((uint32_t)ESP.getEfuseMac(), HEX).substring(0, 4), "bleepbleep");
  web.on("/", [] { web.send_P(200, "text/html", PAGE); });
  web.on("/cmd", [] { cmd[0] = constrain(web.arg("vx").toFloat(), -0.1f, 0.25f); cmd[1] = constrain(web.arg("vy").toFloat(), -0.08f, 0.08f);
                      cmd[2] = constrain(web.arg("wz").toFloat(), -0.8f, 0.8f); web.send(200, "text/plain", "ok"); });
  web.on("/mode", [] { String m = web.arg("m"); if (m == "walk" && state == ST_STAND) setState(ST_WALK); if (m == "stand" && state == ST_WALK) setState(ST_STAND); web.send(200, "text/plain", "ok"); });
  web.begin();
  for (int j = 0; j < N_JOINTS; j++) bus.torque(SERVO_ID[j], true);
  readJoints();
  for (int j = 0; j < N_JOINTS; j++) target[j] = q[j];
  xTaskCreatePinnedToCore(taskSenses, "senses", 8192, nullptr, 1, nullptr, 0);
  snd.happy(); setState(ST_BOOT);
}

void loop() {
  static uint32_t next = micros(); static float boot_from[N_JOINTS]; static bool boot_init = false; static int vcount = 0;
  const float dt = 1.0f / CONTROL_HZ;
  while ((int32_t)(micros() - next) < 0) {}
  next += 1000000 / CONTROL_HZ;
  imu.update(dt);
  readJoints();
  if (++vcount % 100 == 0) { float v = bus.readVoltage(SERVO_ID[NECK_YAW]); if (v > 0) batt_v = 0.9f * batt_v + 0.1f * v; }
  if (state != ST_FALLEN && imu.tilt_deg() > FALL_TILT_DEG) {
    for (int j = 0; j < NECK_YAW; j++) bus.torque(SERVO_ID[j], false);   // go limp: protects servos and fingers
    face.mood = MOOD_SAD; snd.sad(); setState(ST_FALLEN);
  }
  switch (state) {
    case ST_BOOT: {   // ramp from wherever the legs are to the home crouch in 2 s
      if (!boot_init) { for (int j = 0; j < N_JOINTS; j++) boot_from[j] = q[j]; boot_init = true; }
      float u = min(1.0f, (millis() - t_state) / 2000.0f);
      for (int j = 0; j < 10; j++) target[j] = boot_from[j] + u * (Q_HOME[j] - boot_from[j]);
      if (u >= 1.0f) setState(ST_STAND);
      break; }
    case ST_STAND:
      for (int j = 0; j < 10; j++) target[j] = Q_HOME[j];
      for (int j = 0; j < 10; j++) last_act[j] = 0; phase = 0;
      break;
    case ST_WALK: {
      float obs[POLICY_OBS], act[POLICY_ACT], c[3] = {cmd[0], cmd[1], cmd[2]};
      build_obs(obs, imu.g, imu.gyro, c, q, qd, last_act, phase);
      policy_forward(obs, act);
      for (int j = 0; j < 10; j++) { target[j] = Q_HOME[j] + ACTION_SCALE * act[j]; last_act[j] = act[j]; }
      phase += dt / GAIT_PERIOD; if (phase >= 1.0f) phase -= 1.0f;
      break; }
    case ST_FALLEN:   // wait until someone stands it up (a trained get-up policy is future work), then re-boot the pose
      if (imu.tilt_deg() < 15.0f && millis() - t_state > 2000) {
        for (int j = 0; j < N_JOINTS; j++) bus.torque(SERVO_ID[j], true);
        boot_init = false; face.mood = MOOD_HAPPY; snd.happy(); setState(ST_BOOT);
      }
      break;
    case ST_SIT: break;
  }
  target[NECK_YAW] = neck_target;
  if (state != ST_FALLEN) sendTargets();
}
