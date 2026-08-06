// Purpose: Phase 1 firmware for the 6-DOF robot arm — reads six potentiometers
//          and drives the corresponding PCA9685 PWM channels, clamped to safe
//          joint limits with a deadband filter against power-rail noise.
// Author: daamieva
// Date: 2026-08-06
// Dependencies: Arduino.h, Wire.h, math.h, Adafruit_PWMServoDriver.h, config.h

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

// Last angle actually sent to each joint; see DEADBAND_DEG in config.h.
// Sentinel -1 (below any valid clamped angle) forces the first real update.
float lastCommandedDeg[NUM_JOINTS];

float clampAngle(float angle, float minDeg, float maxDeg) {
    if (angle < minDeg) return minDeg;
    if (angle > maxDeg) return maxDeg;
    return angle;
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);   // Init serial for debugging
    pwm.begin();                      // Init PCA9685
    pwm.setPWMFreq(PWM_FREQUENCY_HZ); // Servo refresh rate

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        lastCommandedDeg[i] = -1.0f;
    }
}

void loop() {
    static unsigned long lastPrintTime = 0;
    bool shouldPrint = CALIBRATION_MODE && (millis() - lastPrintTime >= CALIBRATION_PRINT_INTERVAL_MS);

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        const JointConfig &joint = JOINTS[i];

        // In calibration mode, widen the clamp bounds to the full servo
        // range so the joint can be driven to its true mechanical limit.
        float minDeg = CALIBRATION_MODE ? SERVO_ANGLE_MIN_DEG : joint.minAngleDeg;
        float maxDeg = CALIBRATION_MODE ? SERVO_ANGLE_MAX_DEG : joint.maxAngleDeg;

        int potValue = analogRead(joint.analogPin);
        float angleDeg = map(potValue, ADC_MIN, ADC_MAX, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
        float clampedDeg = clampAngle(angleDeg, minDeg, maxDeg);

        // Deadband: hold the last commanded angle unless the new reading
        // moves far enough to be a real input, not power-rail noise.
        if (fabs(clampedDeg - lastCommandedDeg[i]) >= DEADBAND_DEG) {
            lastCommandedDeg[i] = clampedDeg;
        }

        int pulse = SERVO_PULSE_MIN + (lastCommandedDeg[i] / SERVO_ANGLE_MAX_DEG) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);

        pwm.setPWM(joint.pwmChannel, 0, pulse);

        if (shouldPrint) {
            Serial.print(joint.name);
            Serial.print("\tpot=");
            Serial.print(potValue);
            Serial.print("\tangle=");
            Serial.println(lastCommandedDeg[i]);
        }
    }

    if (shouldPrint) {
        Serial.println();
        lastPrintTime = millis();
    }

    delay(LOOP_DELAY_MS); // Small delay for smooth updates
}
