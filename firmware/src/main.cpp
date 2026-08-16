// Purpose: Phase 1 pot-driven joint control plus Phase 3 serial command
//          protocol for the 6-DOF robot arm, including XYZ inverse-kinematics
//          moves. Drives all 6 PCA9685 PWM channels, clamped to safe joint
//          limits with a deadband filter against power-rail noise. Control
//          source (pots vs. serial) is switchable at runtime; see the
//          protocol commands below.
// Author: daamieva
// Date: 2026-08-15
// Dependencies: Arduino.h, Wire.h, math.h, stdlib.h, string.h,
//               Adafruit_PWMServoDriver.h, config.h, kinematics.h
//
// Serial protocol (newline-terminated ASCII, 9600 baud):
//   J<n>:<angle>       -> set joint n (0-5) to angle degrees (SERIAL mode only)
//   XYZ:<x>,<y>,<z>    -> move end-effector to (x,y,z) mm via inverse
//                         kinematics (SERIAL mode only); ERR:UNREACHABLE if
//                         outside the arm's workspace, ERR:PARSE if malformed
//   HOME               -> move all joints to home position (SERIAL mode only)
//   STATE?             -> request current joint angles
//   POTS               -> switch control to potentiometers (default on boot)
//   SERIAL             -> switch control to serial commands
// Replies:
//   STATE:<a0>,<a1>,<a2>,<a3>,<a4>,<a5>   broadcast every SERIAL_PRINT_INTERVAL_MS
//   OK                                     command accepted
//   ERR:<reason>                           command rejected

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "kinematics.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

enum ControlMode { MODE_POTS, MODE_SERIAL };
ControlMode controlMode = MODE_POTS; // default on boot: potentiometers active

// Last angle actually sent to each joint; see DEADBAND_DEG in config.h.
// Sentinel -1 (below any valid clamped angle) forces the first real update.
// Also what STATE? and the periodic broadcast report as current position.
float lastCommandedDeg[NUM_JOINTS];

// Incoming command line buffer for the serial protocol.
static char serialBuffer[32];
static uint8_t serialBufferLen = 0;

float clampAngle(float angle, float minDeg, float maxDeg);
void updatePotJoint(uint8_t i);
void setJoint(uint8_t joint, float angle);
void goHome();
void reportState();
void handleCommand(char *cmd);
void pollSerial();

float clampAngle(float angle, float minDeg, float maxDeg) {
    if (angle < minDeg) return minDeg;
    if (angle > maxDeg) return maxDeg;
    return angle;
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);   // Init serial for the command protocol
    pwm.begin();                      // Init PCA9685
    pwm.setPWMFreq(PWM_FREQUENCY_HZ); // Servo refresh rate

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        lastCommandedDeg[i] = -1.0f;
    }
}

void loop() {
    pollSerial();

    if (controlMode == MODE_POTS) {
        for (uint8_t i = 0; i < NUM_JOINTS; i++) {
            updatePotJoint(i);
        }
    }

    static unsigned long lastBroadcastTime = 0;
    if (millis() - lastBroadcastTime >= SERIAL_PRINT_INTERVAL_MS) {
        reportState();
        lastBroadcastTime = millis();
    }

    delay(LOOP_DELAY_MS); // Small delay for smooth updates
}

// Reads one joint's potentiometer, clamps it through its configured limits,
// and drives its servo. Identical to the original pot-only loop body; only
// pulled into its own function so loop() can gate it on controlMode.
void updatePotJoint(uint8_t i) {
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
}

// Sets one joint to an explicit angle from the serial protocol. Always goes
// through clampAngle() with that joint's real limits — same safety guarantee
// as the pot path. Only takes effect in SERIAL mode, since in POTS mode the
// next pot read (20ms later) would just overwrite it anyway.
void setJoint(uint8_t joint, float angle) {
    if (joint >= NUM_JOINTS) {
        Serial.println("ERR:INVALID_JOINT");
        return;
    }
    if (controlMode != MODE_SERIAL) {
        Serial.println("ERR:WRONG_MODE");
        return;
    }

    const JointConfig &j = JOINTS[joint];
    float clamped = clampAngle(angle, j.minAngleDeg, j.maxAngleDeg);
    lastCommandedDeg[joint] = clamped;

    int pulse = SERVO_PULSE_MIN + (clamped / SERVO_ANGLE_MAX_DEG) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);
    pwm.setPWM(j.pwmChannel, 0, pulse);

    Serial.println("OK");
}

// Moves every joint to a home angle: 90 deg if that's within the joint's
// limits, otherwise the midpoint of its min/max (e.g. wrist rotation's
// 0-75 deg range doesn't include 90, so it homes to 37.5 deg instead).
// Gated to SERIAL mode for the same reason as setJoint() — POTS mode would
// overwrite the home position on the next pot read.
void goHome() {
    if (controlMode != MODE_SERIAL) {
        Serial.println("ERR:WRONG_MODE");
        return;
    }

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        const JointConfig &j = JOINTS[i];
        bool ninetyInRange = (j.minAngleDeg <= 90.0f) && (90.0f <= j.maxAngleDeg);
        float home = ninetyInRange ? 90.0f : (j.minAngleDeg + j.maxAngleDeg) / 2.0f;

        lastCommandedDeg[i] = home;
        int pulse = SERVO_PULSE_MIN + (home / SERVO_ANGLE_MAX_DEG) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);
        pwm.setPWM(j.pwmChannel, 0, pulse);
    }

    Serial.println("OK");
}

// Prints STATE:<a0>,...,<a5> with each angle to one decimal place. Used both
// for the STATE? request and the periodic broadcast in loop().
void reportState() {
    Serial.print("STATE:");
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        Serial.print(lastCommandedDeg[i], 1);
        if (i < NUM_JOINTS - 1) {
            Serial.print(",");
        }
    }
    Serial.println();
}

// Parses one complete command line and dispatches it. setJoint()/goHome()
// send their own OK/ERR replies; every other branch replies here.
void handleCommand(char *cmd) {
    if (cmd[0] == 'J' && strchr(cmd, ':') != nullptr) {
        const char *colon = strchr(cmd, ':');
        int joint = atoi(cmd + 1);     // atoi stops at the first non-digit (':')
        float angle = atof(colon + 1);
        setJoint((uint8_t)joint, angle);
        return;
    }

    if (strncmp(cmd, "XYZ:", 4) == 0) {
        // Gated to SERIAL mode for the same reason as J/HOME: in POTS mode
        // the pot loop would overwrite an IK move within 20ms anyway.
        if (controlMode != MODE_SERIAL) {
            Serial.println("ERR:WRONG_MODE");
            return;
        }

        const char *xText = cmd + 4;
        const char *comma1 = strchr(xText, ',');
        const char *comma2 = (comma1 != nullptr) ? strchr(comma1 + 1, ',') : nullptr;
        if (comma1 == nullptr || comma2 == nullptr) {
            Serial.println("ERR:PARSE");
            return;
        }

        float x = atof(xText);
        float y = atof(comma1 + 1);
        float z = atof(comma2 + 1);

        IKResult result = inverseKinematics(x, y, z, lastCommandedDeg[3]);

        // TEMP DEBUG (Bug 1 verification) — remove before finishing.
        Serial.print("DEBUG fw_shoulder=");
        Serial.print(result.fw_angles[1]);
        Serial.print(" fw_elbow=");
        Serial.println(result.fw_angles[2]);

        if (!result.reachable) {
            Serial.println("ERR:UNREACHABLE");
            return;
        }

        for (uint8_t j = 0; j < NUM_JOINTS; j++) {
            if (j == 3) continue; // gripper is not part of the IK chain
            const JointConfig &joint = JOINTS[j];
            float clamped = clampAngle(result.fw_angles[j], joint.minAngleDeg, joint.maxAngleDeg);
            lastCommandedDeg[j] = clamped;

            int pulse = SERVO_PULSE_MIN + (clamped / SERVO_ANGLE_MAX_DEG) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);
            pwm.setPWM(joint.pwmChannel, 0, pulse);
        }

        Serial.println("OK");
        return;
    }

    if (strcmp(cmd, "HOME") == 0) {
        goHome();
        return;
    }

    if (strcmp(cmd, "STATE?") == 0) {
        reportState();
        return;
    }

    if (strcmp(cmd, "POTS") == 0) {
        controlMode = MODE_POTS;
        Serial.println("OK");
        return;
    }

    if (strcmp(cmd, "SERIAL") == 0) {
        controlMode = MODE_SERIAL;
        Serial.println("OK");
        return;
    }

    Serial.println("ERR:UNKNOWN_COMMAND");
}

// Non-blocking line reader: accumulates characters into serialBuffer and
// dispatches to handleCommand() once a line terminator arrives. Handles
// both \n and \r\n line endings. Input longer than the buffer is silently
// truncated (extra characters dropped) rather than overflowing.
void pollSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBufferLen > 0) {
                serialBuffer[serialBufferLen] = '\0';
                handleCommand(serialBuffer);
                serialBufferLen = 0;
            }
        } else if (serialBufferLen < sizeof(serialBuffer) - 1) {
            serialBuffer[serialBufferLen++] = c;
        }
    }
}
