package first.robot.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    public VisionIOPhotonVisionSim(CameraInfo info, Field field, VisionSystemSim visionSim) {
        super(info, field);
        var properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, org.wpilib.math.geometry.Rotation2d.fromDegrees(70));
        properties.setFPS(30);
        visionSim.addCamera(new PhotonCameraSim(camera(), properties, field), info.robotToCamera());
    }
}
