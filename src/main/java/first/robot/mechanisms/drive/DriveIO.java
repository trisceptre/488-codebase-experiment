package first.robot.mechanisms.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.kinematics.ChassisVelocities;

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

    default void setRobotVelocity(ChassisVelocities velocity) {}

    default void stopWithX() {}

    default void idle() {}
}
