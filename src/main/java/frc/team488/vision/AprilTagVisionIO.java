package frc.team488.vision;

import org.littletonrobotics.junction.AutoLog;

/** One AprilTag camera. Default methods are no-ops (log replay). */
public interface AprilTagVisionIO {
    @AutoLog
    class AprilTagVisionIOInputs {
        public boolean connected = false;
        public VisionPoseObservation[] observations = new VisionPoseObservation[0];
    }

    default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
