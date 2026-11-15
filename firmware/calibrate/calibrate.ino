// Bleep calibration tool (flash this first, then flash firmware/bleep).
// Serial monitor at 115200, line ending "Newline". Commands:
//   scan            list servo IDs found on the bus (1..20)
//   id <old> <new>  change a servo ID (connect ONE new servo at a time: new servos all ship as ID 1)
//   mid <id>        torque off, then store the CURRENT position as the middle (2048) - do this with the joint in the jig at zero
//   free            torque off on all servos (bend the robot by hand)
//   watch           print every joint angle (rad, sim convention with the current JOINT_SIGN) 5x per second; any key stops
//   hold            torque on, hold the zero pose (2048 everywhere) - check that the legs are straight and the head faces forward
#include "config.h"
#include "sts_bus.h"

sts::Bus bus;
static const uint8_t ADDR_ID = 5, ADDR_LOCK = 55;

void writeByte(uint8_t id, uint8_t addr, uint8_t v) { bus.write(id, addr, &v, 1); delay(5); }

void setup() {
  Serial.begin(115200);
  bus.begin(Serial1, PIN_BUS_RX, PIN_BUS_TX, BUS_BAUD);
  delay(500);
  Serial.println("bleep calibrate: scan | id <old> <new> | mid <id> | free | watch | hold");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  int a = -1, b = -1; char w[16] = {0};
  sscanf(cmd.c_str(), "%15s %d %d", w, &a, &b);
  String c(w);
  if (c == "scan") {
    for (uint8_t id = 1; id <= 20; id++) { int16_t p, s; if (bus.readPosSpeed(id, p, s)) Serial.printf("id %d  pos %d  %.1f V\n", id, p, bus.readVoltage(id)); }
    Serial.println("scan done");
  } else if (c == "id" && a > 0 && b > 0 && b < 254) {
    writeByte(a, ADDR_LOCK, 0); writeByte(a, ADDR_ID, b); writeByte(b, ADDR_LOCK, 1);
    Serial.printf("id %d -> %d (run scan to confirm)\n", a, b);
  } else if (c == "mid" && a > 0) {
    bus.torque(a, false); delay(5); writeByte(a, sts::ADDR_TORQUE_ENABLE, 128);   // STS: 128 = calibrate current position to 2048
    Serial.printf("id %d middle set\n", a);
  } else if (c == "free") {
    for (int j = 0; j < N_JOINTS; j++) bus.torque(SERVO_ID[j], false);
    Serial.println("all torque off");
  } else if (c == "hold") {
    uint8_t ids[N_JOINTS]; int16_t pos[N_JOINTS];
    for (int j = 0; j < N_JOINTS; j++) { ids[j] = SERVO_ID[j]; pos[j] = JOINT_ZERO[j]; bus.torque(ids[j], true); }
    bus.syncGoal(ids, pos, N_JOINTS); Serial.println("holding zero pose");
  } else if (c == "watch") {
    while (!Serial.available()) {
      for (int j = 0; j < N_JOINTS; j++) {
        int16_t p, s;
        if (bus.readPosSpeed(SERVO_ID[j], p, s)) Serial.printf("%+.2f ", JOINT_SIGN[j] * (p - JOINT_ZERO[j]) * (2 * PI / 4096));
        else Serial.print("  --  ");
      }
      Serial.println(); delay(200);
    }
    while (Serial.available()) Serial.read();
  } else Serial.println("?");
}
