// Bleep firmware configuration: pins (Seeed XIAO ESP32S3 Sense), servo IDs, joint calibration.
#pragma once
#include <stdint.h>

// ---- XIAO header pins (GPIO numbers)
#define PIN_TOUCH      1    // D0  copper pad inside the dome (ESP32-S3 touch channel 1)
#define PIN_LCD_CS     2    // D1
#define PIN_LCD_DC     3    // D2
#define PIN_BUZZER     4    // D3  passive piezo (LEDC PWM)
#define PIN_SDA        5    // D4  MPU-6050
#define PIN_SCL        6    // D5
#define PIN_BUS_TX     43   // D6  -> Waveshare Bus Servo Adapter (A) RX
#define PIN_BUS_RX     44   // D7  <- adapter TX
#define PIN_LCD_SCK    7    // D8
#define PIN_LCD_RST    8    // D9
#define PIN_LCD_MOSI   9    // D10
// on-board (Sense expansion board): PDM microphone CLK 42 / DATA 41, camera on its own pins (see camera.h)
#define PIN_MIC_CLK    42
#define PIN_MIC_DATA   41

#define BUS_BAUD       1000000
#define CONTROL_HZ     50

// ---- joints. Index order = policy order (sim/env.py LEG) + neck.
enum { L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE, R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE, NECK_YAW, N_JOINTS };
static const uint8_t SERVO_ID[N_JOINTS]  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
// +1/-1: maps servo count direction to the sim joint direction. Fill in with firmware/calibrate "watch" (see docs/ASSEMBLY.md step 8).
static const int8_t  JOINT_SIGN[N_JOINTS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
// servo count that equals sim joint angle 0 (set with Feetech "set middle" in the assembly jig -> 2048)
static const int16_t JOINT_ZERO[N_JOINTS] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
// joint limits (rad), same as cad/bleep.py LIMITS_DEG
static const float Q_MIN[N_JOINTS] = {-0.436f, -0.349f, -1.571f, -0.087f, -0.785f, -0.436f, -0.349f, -1.571f, -0.087f, -0.785f, -2.094f};
static const float Q_MAX[N_JOINTS] = { 0.436f,  0.349f,  0.524f,  1.920f,  0.785f,  0.436f,  0.349f,  0.524f,  1.920f,  0.785f,  2.094f};

#define FALL_TILT_DEG  50.0f     // tilt beyond this = fallen: relax legs, sad face
#define LOW_BATT_V     6.0f      // 2S LiFePO4: 6.0 V = ~10 % left
