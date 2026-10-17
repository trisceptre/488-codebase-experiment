package first.robot.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

    default void updateInputs(VisionIOInputs inputs) {}
}
