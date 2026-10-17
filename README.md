# Robot Template

Team 488's robot template on WPILib 2026 (roboRIO, Java 17, Commands v2), running the 2026 robot.

## Robots

`RobotIdentity` picks the robot from the roboRIO serial number; unknown serials run as `COMP_2026`.

| Identity | Robot | Notes |
|---|---|---|
| `COMP_2026` | 2026 competition robot | `TunerConstants` |
| `CHASSIS_2025` | 2025 robot, drive only | `TunerConstants2025`; no lights (DIO-driven) |
| `SIM` | Simulation | Uses the 2026 config |

## Pre-season Checklist

1. **WPILib & vendordeps**: bump GradleRIO, `frcYear` in `settings.gradle`, `projectYear`, and update vendordeps.
2. **Drivetrain**: regenerate tuner constants in Tuner X.
3. **Encoder offsets**: read each CANcoder offset in Tuner X and set `k*EncoderOffset` in `TunerConstants` (and `TunerConstants2025`). The robot raises "CANcoder offsets not set in TunerConstants" while any is `0.0`.
4. **Robot identity**: add each roboRIO serial number to `BY_SERIAL` in `constants/RobotIdentity.java`.
5. **Field**: set the field in `constants/RobotConfigs.java`.
6. **Cameras**: update the camera lists in `constants/RobotConfigs.java`. Photon camera names must match the coprocessor.
7. **Lights & power**: set the CANdle ID, bus and LED count, and the PDH CAN ID in `constants/RobotConfigs.java`.
8. **Wheel radius**: on the debug controller (port 2), hold A while the robot spins in place. Copy `Drive/WheelRadiusCharacterization/RadiusMeters` into `kWheelRadius` in `TunerConstants`.
9. **Tuning**: copy final tunable values back into the code defaults.

## Building

Gradle runs on JDK 17 or 21 (WPILib 2026 bundles 17). Bytecode is always Java 17 for the roboRIO.
