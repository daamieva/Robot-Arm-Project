// Purpose: Forward and inverse kinematics for the 6-DOF arm.
// Author:  daamieva
// Date:    2026-08-15
// Dependencies: Arduino.h, math.h
//
// Coordinate system:
//   Origin: base rotation axis at board level
//   +X: forward (arm faces +X at base firmware angle 90 deg)
//   +Y: left when standing behind the arm
//   +Z: up
//
// All positions in millimetres. All angles in degrees (firmware domain).
// Firmware domain: the angle values reported by the serial monitor.
// Geometric domain: angles used internally in the math (radians, zero at home).

#pragma once
#include <Arduino.h>

// Result struct returned by IK
struct IKResult {
    bool reachable;          // false if target is outside workspace
    float fw_angles[6];      // firmware angles for joints 0-5 (degrees)
                             // index 3 (gripper) is unchanged from current
};

// inverseKinematics
// Given a target gripper tip position (x, y, z) in mm,
// computes firmware joint angles to reach that position.
// current_gripper_deg: current gripper angle (preserved in result)
// Returns IKResult with reachable=false if target is outside workspace.
IKResult inverseKinematics(float x, float y, float z,
                            float current_gripper_deg);

// forwardKinematics
// Given firmware joint angles, computes gripper tip position.
// fw_angles: array of 6 firmware angles in degrees
// out_x, out_y, out_z: output position in mm
void forwardKinematics(const float fw_angles[6],
                        float* out_x, float* out_y, float* out_z);
