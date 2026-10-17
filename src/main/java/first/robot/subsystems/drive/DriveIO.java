package first.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

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

    default void setRobotVelocity(ChassisSpeeds velocity) {}

    default void stopWithX() {}

    default void idle() {}

    default void resetPose(Pose2d pose) {}
}
