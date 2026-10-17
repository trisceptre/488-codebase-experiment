package first.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    public VisionIOPhotonVisionSim(CameraInfo info, AprilTagFieldLayout field, VisionSystemSim visionSim) {
        super(info, field);
        var properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        properties.setFPS(30);
        visionSim.addCamera(new PhotonCameraSim(camera(), properties, field), info.robotToCamera());
    }
}
