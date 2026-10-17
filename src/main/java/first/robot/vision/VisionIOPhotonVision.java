package first.robot.vision;

import first.robot.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;

public class VisionIOPhotonVision implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public VisionIOPhotonVision(CameraInfo info, Field field) {
        camera = new PhotonCamera(info.photonCameraName());
        estimator = new PhotonPoseEstimator(field, info.robotToCamera());
    }

    protected PhotonCamera camera() {
        return camera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        List<PoseObservation> observations = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            var estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            estimate.ifPresent(e -> observations.add(toObservation(e)));
        }
        inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    }

    private static PoseObservation toObservation(EstimatedRobotPose estimate) {
        List<PhotonTrackedTarget> targets = estimate.targetsUsed;
        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        int count = targets.size();
        double ambiguity = count == 1 ? targets.getFirst().getPoseAmbiguity() : 0.0;
        return new PoseObservation(
                estimate.timestampSeconds,
                estimate.estimatedPose,
                ambiguity,
                count,
                count == 0 ? 0.0 : totalDistance / count);
    }
}
