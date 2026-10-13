# Robot Template

## Pre-season Checklist

1. **WPILib & vendordeps**: bump GradleRIO, `wpilibYear`, `projectYear`, and update vendordeps.
2. **Drivetrain**: regenerate tuner constants in Tuner X.
3. **Encoder offsets**: read each CANcoder offset in Tuner X and set `k*EncoderOffset` in `TunerConstants`. The robot will raise `Drive/OffsetsMissing` while any is `0.0`.
4. **Robot identity**: add the controller serial number to `BY_SERIAL` in `constants/RobotIdentity.java`.
5. **Field**: set the field in `constants/RobotConfigs.java`.
6. **Cameras**: update `CAMERAS` in `constants/RobotConfigs.java`.
7. **Lights & power**: set CANdle ID, bus and LED count, and the PDH `CANPort` in `constants/RobotConfigs.java`.
8. **Wheel radius**: on the debug controller (port 2), hold A while the robot spins in place. Copy `Drive/WheelRadiusCharacterization/RadiusMeters` into `kWheelRadius` in `TunerConstants`.
9. **Tuning**: copy final tunable values back into the code defaults.
