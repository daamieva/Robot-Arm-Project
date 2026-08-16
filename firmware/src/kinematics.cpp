// Purpose: Forward and inverse kinematics for the 6-DOF arm — geometric
//          closed-form solve for a yaw base + 3R pitch chain (shoulder,
//          elbow, wrist swivel) + roll wrist, per the measured arm geometry
//          below (validated against a MATLAB rigidBodyTree model).
// Author:  daamieva
// Date:    2026-08-15
// Dependencies: Arduino.h, math.h, kinematics.h
//
// This file is intentionally self-contained: it does not include config.h.
// The geometry constants below are specific to the physical kinematic model
// (measured link lengths, home angles), not general firmware configuration,
// so they're kept local rather than mixed in with pin/PWM/limit constants.

#include <Arduino.h>
#include <math.h>
#include "kinematics.h"

// ---- Measured arm geometry (mm). Validated against a MATLAB rigidBodyTree
// model across 5 physical test poses — do not change without re-measuring. ----
static const float H_BASE = 90.0f;    // board to shoulder axis, vertical height
static const float R_OFFSET = 21.5f;  // shoulder axis offset behind base center
static const float L2 = 113.0f;       // shoulder axis to elbow axis
static const float L3 = 127.5f;       // elbow axis to wrist axis
static const float L4 = 100.0f;       // wrist axis to gripper tip

// ---- Firmware home angles (degrees). Geometric zero for every joint is
// defined as this home configuration. ----
static const float FW_HOME_BASE_DEG = 90.0f;
static const float FW_HOME_SHOULDER_DEG = 68.0f;
static const float FW_HOME_ELBOW_DEG = 91.0f;
static const float FW_HOME_WRIST_SWIVEL_DEG = 138.0f;
static const float FW_HOME_WRIST_ROTATION_DEG = 30.0f;

IKResult inverseKinematics(float x, float y, float z, float current_gripper_deg) {
    IKResult result;
    result.reachable = false;
    result.fw_angles[3] = current_gripper_deg; // gripper isn't part of the IK chain

    // ---- Stage 1: base yaw ----
    // The base only rotates about Z, so its angle is just the direction
    // from the base axis to the target, measured in the XY plane.
    float phi = atan2(y, x);
    result.fw_angles[0] = phi * RAD_TO_DEG + FW_HOME_BASE_DEG;

    // ---- Stage 2: planar geometry ----
    // The shoulder axis isn't at the base center — it sits R_OFFSET behind
    // it, and that offset swings around with the base as it yaws.
    float shoulder_x = -R_OFFSET * cos(phi);
    float shoulder_y = -R_OFFSET * sin(phi);
    float shoulder_z = H_BASE;

    // Target position relative to the shoulder axis: this is what the
    // shoulder/elbow 2-link chain actually has to reach.
    float dx = x - shoulder_x;
    float dy = y - shoulder_y;
    float dz = z - shoulder_z;

    // Horizontal distance from the shoulder to the target, and the full
    // straight-line 3D distance the arm has to span.
    float r_horiz = sqrt(dx * dx + dy * dy);
    float r_total = sqrt(r_horiz * r_horiz + dz * dz);

    // A target farther than the arm's full extended reach, or closer than
    // the shortest span the two main links can fold down to, is impossible.
    if (r_total > (L2 + L3 + L4)) {
        return result; // reachable stays false
    }
    if (r_total < fabs(L2 - L3)) {
        return result;
    }

    // ---- Stage 3: elbow angle ----
    // This first-pass model doesn't solve wrist orientation (see Stage 5),
    // so it treats the wrist and gripper as pointing straight out along the
    // forearm. The gripper's length is removed from the target reach first,
    // leaving the distance the shoulder/elbow 2-link chain must span to
    // place the wrist correctly.
    float r_wrist_reach = r_total - L4;

    // Guard against a degenerate reach (e.g. a target close enough to the
    // shoulder that subtracting the gripper length leaves ~0 or negative
    // distance) before it hits the law-of-cosines math below, where it
    // would otherwise produce a divide-by-zero or an out-of-domain acos().
    // This tightens the coarser check above to the actual 2-link (L2, L3)
    // domain the elbow/shoulder solve below depends on.
    if (r_wrist_reach <= 0.0001f || r_wrist_reach > (L2 + L3)) {
        return result;
    }

    // Law of cosines: the INTERIOR angle at the elbow vertex between the
    // upper arm (L2) and forearm (L3) that makes the two links span
    // exactly r_wrist_reach. This is 180 deg (PI) when the arm is fully
    // straight (collinear links) and shrinks toward 0 as it folds up.
    float cos_elbow_geo = (L2 * L2 + L3 * L3 - r_wrist_reach * r_wrist_reach) / (2.0f * L2 * L3);
    // Floating-point error can push this a hair outside [-1, 1] right at
    // the extremes of reach, which would make acos() return NaN.
    if (cos_elbow_geo > 1.0f) cos_elbow_geo = 1.0f;
    if (cos_elbow_geo < -1.0f) cos_elbow_geo = -1.0f;
    float elbow_geo = acos(cos_elbow_geo);

    // elbow_geo is an interior angle (180 deg = straight), but "geometric
    // zero" for this joint is defined as straight (forearm collinear with
    // upper arm) — so it has to be converted to a bend-away-from-straight
    // angle (0 = straight, growing as the arm folds) before it's used.
    float elbow_bend = PI - elbow_geo;

    // The elbow's physical rotation axis is -Y (opposite the shoulder's
    // +Y), so a positive bend here is a DEcreasing firmware angle away
    // from its 91 deg home.
    result.fw_angles[2] = FW_HOME_ELBOW_DEG - elbow_bend * RAD_TO_DEG;

    // ---- Stage 4: shoulder angle ----
    // alpha is the elevation angle from the shoulder to the target,
    // measured from horizontal (atan2(dz, r_horiz) is 0 when horizontal,
    // 90 deg when straight up) — but shoulder_geo is measured from
    // vertical (68 deg home = vertical = geometric zero), so alpha has to
    // be converted to that same reference before it's used for anything.
    float alpha = atan2(dz, r_horiz);                // angle from horizontal
    float alpha_from_vertical = (PI / 2.0f) - alpha;  // convert reference

    float cos_beta = (L2 * L2 + r_wrist_reach * r_wrist_reach - L3 * L3) / (2.0f * L2 * r_wrist_reach);
    // Same floating-point safety margin as the elbow's acos() above.
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acos(cos_beta); // law of cosines term, unchanged

    // alpha_from_vertical is the elevation from vertical to the target
    // direction; beta is the angle the upper arm makes above that line to
    // reach the elbow correctly, so the shoulder must sit above the
    // target line by beta.
    float shoulder_geo = alpha_from_vertical - beta;

    // Positive firmware shoulder angle tilts the arm forward, the same
    // direction as increasing geometric angle, so this is a direct offset
    // from the 68 deg home.
    result.fw_angles[1] = FW_HOME_SHOULDER_DEG + shoulder_geo * RAD_TO_DEG;

    // ---- Stage 5: wrist ----
    // This initial implementation doesn't solve wrist orientation — both
    // wrist joints just hold their home angles. Extending this to aim the
    // wrist deliberately is future work.
    result.fw_angles[4] = FW_HOME_WRIST_SWIVEL_DEG;
    result.fw_angles[5] = FW_HOME_WRIST_ROTATION_DEG;

    // ---- Stage 6: gripper ----
    // Already set above (result.fw_angles[3] = current_gripper_deg) — the
    // gripper isn't part of the kinematic chain.

    result.reachable = true;
    return result;
}

void forwardKinematics(const float fw_angles[6], float* out_x, float* out_y, float* out_z) {
    // Step 1: base yaw. At firmware angle 90 deg the arm faces +X (phi = 0),
    // so subtracting 90 converts the firmware reading to a geometric yaw.
    float phi = (fw_angles[0] - FW_HOME_BASE_DEG) * DEG_TO_RAD;

    // Step 3: shoulder pitch. Firmware home is 68 deg = vertical (geometric
    // zero); positive geometric pitch tilts the arm toward +X.
    float theta2 = (fw_angles[1] - FW_HOME_SHOULDER_DEG) * DEG_TO_RAD;

    // Step 4: elbow pitch. The elbow's physical axis is -Y (opposite the
    // shoulder's +Y) and firmware home is 91 deg, so its contribution to
    // the same cumulative pitch is the negated offset from home.
    float theta3 = -(fw_angles[2] - FW_HOME_ELBOW_DEG) * DEG_TO_RAD;

    // Direction of the upper arm (shoulder to elbow) in the local,
    // pre-base-yaw vertical plane: a unit vector tilted by theta2 away from
    // straight up (+Z).
    float dirL2_x = sin(theta2);
    float dirL2_z = cos(theta2);

    // Step 5/6: direction of the forearm (elbow to wrist) and the gripper
    // extension (wrist to tip). Both run along the same cumulative
    // shoulder+elbow pitch direction — this simplified model doesn't apply
    // wrist swivel's own pitch separately (it mirrors the IK simplification
    // in Stage 3/5 above, which holds wrist swivel at home).
    float cumulative = theta2 + theta3;
    float dirL3_x = sin(cumulative);
    float dirL3_z = cos(cumulative);

    // Step 2: shoulder axis offset. In the local (pre-yaw) frame the
    // shoulder sits R_OFFSET behind the base center (negative local X) and
    // H_BASE above the board.
    float local_x = -R_OFFSET + L2 * dirL2_x + (L3 + L4) * dirL3_x;
    float local_z = H_BASE + L2 * dirL2_z + (L3 + L4) * dirL3_z;
    // No local_y term: every segment above lies in the local XZ plane —
    // Steps 5/6 run L3 and L4 "along the arm axis", which never leaves that
    // plane until the base yaw below is applied.

    // Step 1 (applied last): rotate the whole local-frame chain by the base
    // yaw to get world coordinates. Z is unaffected by a rotation about Z.
    *out_x = local_x * cos(phi);
    *out_y = local_x * sin(phi);
    *out_z = local_z;
}
