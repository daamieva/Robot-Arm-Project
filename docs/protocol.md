# Serial Protocol

The Arduino firmware ([firmware/src/main.cpp](../firmware/src/main.cpp)) and the Python host
([software/arm_control.py](../software/arm_control.py)) communicate over a newline-terminated
ASCII protocol at 9600 baud. Every command can be tested manually from the Arduino Serial
Monitor — no Python required.

## Commands (PC → Arduino)

| Command | Format | Description |
|---|---|---|
| Set joint | `J<n>:<angle>` | Set joint `n` (0-5) to `angle` degrees. Requires `SERIAL` mode — replies `ERR:WRONG_MODE` otherwise. Out-of-range angles are silently clamped to that joint's configured limits, not rejected. |
| Reserved (IK) | `XYZ:<x>,<y>,<z>` | Reserved for future inverse-kinematics control. Currently parsed but always replies `ERR:NOT_IMPLEMENTED`. |
| Home | `HOME` | Move all joints to their home position. Requires `SERIAL` mode. |
| Query state | `STATE?` | Request the current joint angles immediately. |
| Pots mode | `POTS` | Switch control to the potentiometers (default on boot). |
| Serial mode | `SERIAL` | Switch control to serial commands; potentiometers are ignored until `POTS` is sent again. |

## Replies (Arduino → PC)

| Reply | Format | When |
|---|---|---|
| State broadcast | `STATE:<a0>,<a1>,<a2>,<a3>,<a4>,<a5>` | Automatically every 200ms, regardless of control mode, and also in direct response to `STATE?`. Each angle is a float to 1 decimal place. |
| Accepted | `OK` | A command was valid and applied. |
| Rejected | `ERR:<reason>` | A command was invalid or refused — see reasons below. |

## Error reasons

| Reason | Meaning |
|---|---|
| `INVALID_JOINT` | Joint index in a `J<n>:<angle>` command was outside 0-5. |
| `WRONG_MODE` | A `J<n>:<angle>` or `HOME` command was sent while in `POTS` mode. Send `SERIAL` first. |
| `NOT_IMPLEMENTED` | An `XYZ:...` command was sent; inverse kinematics isn't built yet. |
| `UNKNOWN_COMMAND` | The line didn't match any known command. |

## Joint index mapping

| Index | Joint | Limits (deg) |
|---|---|---|
| 0 | Base swivel | 0–180 |
| 1 | Shoulder | 0–160 |
| 2 | Elbow | 0–180 |
| 3 | Gripper | 40–115 |
| 4 | Wrist swivel | 0–180 |
| 5 | Wrist rotation | 0–75 |

Index 3 (gripper) is wired to PCA9685 channel 6 in the firmware — a hardware wiring exception
documented in [firmware/include/config.h](../firmware/include/config.h). The serial protocol only
ever addresses joints by index; callers never need to know about PWM channel numbers.

Home position is 90° for every joint except wrist rotation, which homes to 37.5° (the midpoint
of its 0–75° range, since 90° falls outside it).

## Example session

```
> SERIAL
< OK
> J2:90
< OK
> STATE?
< STATE:89.0,67.0,90.0,40.0,139.0,31.0
> HOME
< OK
> POTS
< OK
```
