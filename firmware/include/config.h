// Purpose: Hardware constants and joint table for the 6-DOF robot arm firmware.
// Author: daamieva
// Date: 2026-08-04
// Dependencies: Arduino.h
#pragma once

#include <Arduino.h>

// Serial
static constexpr long SERIAL_BAUD_RATE = 9600;

// I2C (Arduino Mega hardware I2C pins, fixed by silicon; documentation only —
// not passed to Wire.begin(), which uses these pins implicitly)
static constexpr uint8_t I2C_SDA_PIN = 20;
static constexpr uint8_t I2C_SCL_PIN = 21;

// PCA9685 PWM driver
static constexpr uint8_t PCA9685_I2C_ADDR = 0x40;
static constexpr float PWM_FREQUENCY_HZ = 60;

// Servo pulse range (PCA9685 ticks) and the angle range they represent
static constexpr int SERVO_PULSE_MIN = 125;
static constexpr int SERVO_PULSE_MAX = 575;
static constexpr float SERVO_ANGLE_MIN_DEG = 0.0f;
static constexpr float SERVO_ANGLE_MAX_DEG = 180.0f;

// Potentiometer ADC range
static constexpr int ADC_MIN = 0;
static constexpr int ADC_MAX = 1023;

// Main loop timing
static constexpr int LOOP_DELAY_MS = 20;

// Calibration mode: when true, joint angle limits below are ignored (servos
// can travel the full 0-180 deg range) and each joint's pot reading + angle
// is printed to Serial so mechanical limits can be found by hand. Flip back
// to false once JOINT_LIMIT_MIN_DEG/MAX_DEG and the per-joint limits in
// JOINTS[] below are set to measured values.
static constexpr bool CALIBRATION_MODE = false;
static constexpr unsigned long CALIBRATION_PRINT_INTERVAL_MS = 200;

// Deadband filter: ignore angle changes smaller than DEADBAND_DEG. This is a
// temporary software workaround for cross-talk between joints — all 6 servos
// currently share one underdimensioned power rail, so the current draw of
// one servo moving sags that rail (and the Arduino's analog reference),
// making other joints' pot readings jitter and their servos visibly twitch.
// The real fix is a dedicated, decoupled servo power supply with a common
// ground back to the Mega; remove this filter once that's in place.
static constexpr float DEADBAND_DEG = 2.0f;

// Measured joint limits (degrees, in the same 0-180 label domain the pot
// is mapped into — see main.cpp). Base swivel, elbow, and wrist swivel have
// no linkage restriction beyond their servo's own throw, so they use the
// full SERVO_ANGLE_MIN_DEG..SERVO_ANGLE_MAX_DEG range directly.
static constexpr float SHOULDER_LIMIT_MIN_DEG = 0.0f;
static constexpr float SHOULDER_LIMIT_MAX_DEG = 160.0f;
static constexpr float GRIPPER_LIMIT_MIN_DEG = 40.0f;
static constexpr float GRIPPER_LIMIT_MAX_DEG = 115.0f;
static constexpr float WRIST_ROTATION_LIMIT_MIN_DEG = 0.0f;
static constexpr float WRIST_ROTATION_LIMIT_MAX_DEG = 75.0f;

// Link lengths (mm) — TODO: measure and set; unused until Phase 2 kinematics
static constexpr float LINK_BASE_TO_SHOULDER_MM = 0.0f;
static constexpr float LINK_SHOULDER_TO_ELBOW_MM = 0.0f;
static constexpr float LINK_ELBOW_TO_WRIST_MM = 0.0f;
static constexpr float LINK_WRIST_TO_GRIPPER_MM = 0.0f;

// Joint table: one entry per servo, base to tip.
// Channel 6 for the gripper (index 3) is a hardware wiring exception —
// every other joint's PCA9685 channel matches its array index.
struct JointConfig {
    uint8_t analogPin;
    uint8_t pwmChannel;
    float minAngleDeg;
    float maxAngleDeg;
    const char* name;
};

static constexpr uint8_t NUM_JOINTS = 6;

static const JointConfig JOINTS[NUM_JOINTS] = {
    { A0, 0, SERVO_ANGLE_MIN_DEG,        SERVO_ANGLE_MAX_DEG,        "base swivel"    },
    { A1, 1, SHOULDER_LIMIT_MIN_DEG,     SHOULDER_LIMIT_MAX_DEG,     "shoulder"       },
    { A2, 2, SERVO_ANGLE_MIN_DEG,        SERVO_ANGLE_MAX_DEG,        "elbow"          },
    { A3, 6, GRIPPER_LIMIT_MIN_DEG,      GRIPPER_LIMIT_MAX_DEG,      "gripper"        },
    { A4, 4, SERVO_ANGLE_MIN_DEG,        SERVO_ANGLE_MAX_DEG,        "wrist swivel"   },
    { A5, 5, WRIST_ROTATION_LIMIT_MIN_DEG, WRIST_ROTATION_LIMIT_MAX_DEG, "wrist rotation" },
};
