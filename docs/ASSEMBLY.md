# Assembly guide

Expect about 2-3 hours with a soldering iron. Do the steps in order: servo IDs and middle positions are much easier to set before anything is screwed together.

**Tools:** soldering iron (also used for the heat-set inserts), 1.5 / 2 / 2.5 mm hex keys, a small Phillips driver, flush cutters, a multimeter, a USB-C cable.

**Fasteners:**
- 44 x M2x8 self-tapping screws: case screws, 4 per servo.
- 44 x M3x6: hub-to-horn screws, 4 per hub.
- 4 x M3 heat-set inserts and 4 x M3x8: lid.
- 3 x M2x6: head shell to head base.
- 2 x M2x4: XIAO to its post.

Joint numbering, which is also the servo ID:

| ID | Joint | ID | Joint |
|---|---|---|---|
| 1 | left hip yaw | 6 | right hip yaw |
| 2 | left hip roll | 7 | right hip roll |
| 3 | left hip pitch | 8 | right hip pitch |
| 4 | left knee | 9 | right knee |
| 5 | left ankle | 10 | right ankle |
| 11 | dome (neck) yaw | | |

Left and right are the robot's own left and right.

## 1. Program the servo IDs and middles (before assembly)
1. Flash `firmware/calibrate/` to the XIAO (Arduino IDE or `arduino-cli`, board **XIAO_ESP32S3**, PSRAM **OPI**).
2. Wire XIAO D6 to the adapter's RX, D7 to its TX, and GND to GND. Power the adapter from the battery pack, or from a 6-7.4 V bench supply.
3. Plug in **one servo at a time**. Type `id 1 <n>`, using the table above. Label each servo with tape.
4. With each servo still loose, type `mid <n>`. This makes the servo's current position count as 2048. Do it with the horn removed, then refit the horn so its 4 screw holes are square to the case.
   - A loose servo's current position is arbitrary. For the legs, step 7 resets the middles in the zero pose, which is what matters.

## 2. Inserts
Melt the 4 M3 heat-set inserts into the corner bosses of `body_tub`.

## 3. Legs (do the left leg, then mirror for the right)
Each servo slides into its pod from the open (horn) side, and 4 M2x8 screws go through the back wall into the case. The cable exits through the slot in the tail wall.

1. **Ankle (ID 5) into the shin.** Screw the `foot` hub plate onto the ankle horn with 4 M3x6, with the foot pointing forward and the shin vertical.
2. **Knee (ID 4) into the thigh.** Screw the `shin` hub onto the knee horn, with the shin straight below the thigh.
3. **Hip pitch (ID 3) into the `hip_carrier`.** Screw the `thigh` hub onto the pitch horn, with the leg straight down.
4. **Hip roll (ID 2) into the `hip_bracket`.** Screw the `hip_carrier` hub onto the roll horn (it faces forward), with the leg vertical.
5. Chain the servo cables ankle -> knee -> pitch -> roll, using the cables that came with the servos. Leave the last one free.

## 4. Body tub
1. **Hip yaw servos (IDs 1 and 6)** go into the two pods under the tub, with the horns facing down.
2. Screw each leg's `hip_bracket` hub onto its yaw horn, with the leg pointing straight forward.
3. Route each leg's chain cable up into the tub.
4. **Cells:** solder the two LiFePO4 cells in series through the 2S BMS: B- / BM / B+. Check with the multimeter: 6.4-7.3 V at P+ / P-.
   - Lay the cells in the two cradles along the floor of the tub.
5. **Rear panel:** fit the DC jack and the slide switch.
   - Wire: jack -> BMS P+/P- (charge in), and pack -> switch -> adapter power input.
6. **Power to the head:** run the switched pack voltage (VBAT, GND) up the neck harness. The Pololu D36V6F5 5 V regulator lives in the head (step 6).
7. **Adapter and IMU:** stick the bus adapter on the front wall. Screw or glue the MPU-6050 flat against the front wall: x forward, z up, and **rigid** (no foam).
8. Plug both leg chains into the adapter.

## 5. Lid and dome yaw
1. Put the **dome servo (ID 11)** in the lid pod, with the horn up.
2. Run the neck harness through the two lid pass-throughs and leave a **10 cm service loop** under the lid, because the dome turns +-120 degrees. The harness carries:
   - VBAT and GND;
   - the servo bus (3 wires);
   - IMU I2C (SDA, SCL);
   - the bus UART (TX, RX).
3. Screw the lid to the tub with 4 M3x8.

## 6. Head
1. Screw `head_base` onto the dome horn with 4 M3x6, with the eye facing forward.
   Stick the Pololu D36V6F5 to the base beside the LCD cradle: VIN to VBAT, VOUT to the XIAO 5V pin and the LCD VCC.
2. **LCD** into the round cradle. Wire it to the XIAO:

   | LCD | XIAO pin |
   |---|---|
   | SCK | D8 |
   | MOSI | D10 |
   | CS | D1 |
   | DC | D2 |
   | RST | D9 |
   | VCC | 5 V |
   | GND | GND |
   | BL | 3V3 |

3. **XIAO ESP32S3 Sense** on its post: camera board fitted, lens toward the camera port. Wire:
   - IMU SDA to D4, SCL to D5;
   - bus TX to D6 (to the adapter RX) and RX to D7 (from the adapter TX);
   - buzzer to D3 (other leg to GND);
   - touch pad to D0.
4. **Touch pad:** stick a 40 x 40 mm square of copper tape inside the top of the dome. Solder a wire from it to D0.
5. Close the head shell onto the base with 3 x M2x6.

## 7. Set the true zero pose
1. Flash `firmware/calibrate` and switch on.
2. Type `free`, then put the robot in the zero pose:
   - legs straight and parallel;
   - feet flat and pointing forward;
   - hips not yawed or rolled;
   - head facing forward.
3. Hold it there (lying on its back is easiest) and type `mid 1` through `mid 11`.
4. Type `hold`: the robot should hold that pose.

## 8. Joint signs
1. Type `watch`. Bend each joint by hand in the **positive** direction:
   - hip yaw: toes turn left;
   - hip roll: leg swings out (left leg) or in (right leg); this is a rotation about x forward;
   - hip pitch: thigh swings **back**;
   - knee: shin swings back (bent knee positive);
   - ankle: toe **down**;
   - dome: turns left.
2. Every number should go **up**. Where one goes down, set that joint's entry in `JOINT_SIGN` in `firmware/bleep/config.h` to -1.
   - Mirrored servos on the right leg usually need -1 on the pitch, knee and ankle.

## 9. Flash the robot firmware
1. Flash `firmware/bleep` and switch on, with the robot held upright with the feet on a table.
2. It ramps to the home crouch in 2 s and shows a face.
3. Join the Wi-Fi network `Bleep-xxxx` (password `bleepbleep`) and open `http://192.168.4.1` to drive it.
   - Or tap the top of the dome to start or stop walking.
4. Before the first walk, do step 10.

## 10. System ID (strongly recommended)
Before the first walk, measure one of your servos on the desk rig described in `sim/sysid.py`:
1. Clamp one spare-or-unmounted STS3215 to the desk.
2. Bolt a 100 mm arm with a 200 g weight to its horn.
3. Connect it to a PC through the adapter's USB-C port.
4. Run `python sim/sysid.py record ...` and then `python sim/sysid.py fit ...`.

This measures the real gain, damping, friction and stall torque. Put the values into `sim/servo_prior.json`, then fine-tune the policy (see `sim/README.md`).

## Safety
- LiFePO4 only, with the BMS, charged with a **7.3 V LiFePO4** charger.
- Never fit 18650 Li-ion cells: 8.4 V when full is above the servos' rating.
- Switch off before plugging or unplugging servo cables.
- The robot goes limp when it detects a fall (tilt over 50 degrees). Stand it back up and it restarts by itself.
