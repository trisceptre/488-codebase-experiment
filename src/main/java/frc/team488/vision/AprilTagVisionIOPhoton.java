package frc.team488.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;

/** A PhotonVision camera. Multi-tag on the coprocessor, falling back to the least ambiguous single tag. */
public class AprilTagVisionIOPhoton implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public AprilTagVisionIOPhoton(CameraInfo info, Field field) {
        camera = new PhotonCamera(info.photonCameraName());
        estimator = new PhotonPoseEstimator(field, info.robotToCamera());
    }

    protected PhotonCamera camera() {
        return camera;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        List<VisionPoseObservation> observations = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            var estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            estimate.ifPresent(e -> observations.add(toObservation(e)));
        }
        inputs.observations = observations.toArray(new VisionPoseObservation[0]);
    }

    private static VisionPoseObservation toObservation(EstimatedRobotPose estimate) {
        List<PhotonTrackedTarget> targets = estimate.targetsUsed;
        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        int count = targets.size();
        double ambiguity = count == 1 ? targets.getFirst().getPoseAmbiguity() : 0.0;
        return new VisionPoseObservation(
                estimate.timestampSeconds,
                estimate.estimatedPose,
                ambiguity,
                count,
                count == 0 ? 0.0 : totalDistance / count);
    }
}
