package frc.team488.mechanisms.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Hardware boundary for the drivetrain. Odometry arrives as every sample taken since the last loop
 * (flattened: sample i, module j at index {@code i * moduleCount + j}), so replay sees exactly what
 * the robot saw. Default methods are no-ops, the IO used during log replay.
 */
public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryYawRad = new double[] {};
        public double[] odometryModuleDistancesMeters = new double[] {};
        public double[] odometryModuleAnglesRad = new double[] {};
        public double[] moduleSpeedsMetersPerSec = new double[] {};
        public double[] moduleAnglesRad = new double[] {};
        public boolean[] driveConnected = new boolean[] {};
        public boolean[] steerConnected = new boolean[] {};
        public boolean[] encoderConnected = new boolean[] {};
        public boolean gyroConnected = false;
        public long droppedOdometrySamples = 0;
    }

    default void updateInputs(DriveIOInputs inputs) {}

    /** Drive at a robot-relative velocity. */
    default void setRobotVelocity(ChassisVelocities velocity) {}

    /** Point the wheels inward (the "X-lock" stance) to resist being pushed. */
    default void lockWheels() {}

    /** Stop driving and let the modules coast or brake per their neutral mode. */
    default void idle() {}
}
