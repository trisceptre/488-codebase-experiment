package frc.team488.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;

/**
 * A simulated Photon camera. Registers with a shared {@link VisionSystemSim}; Robot updates that
 * once per loop with the simulated drivetrain's true pose.
 */
public class AprilTagVisionIOSim extends AprilTagVisionIOPhoton {
    public AprilTagVisionIOSim(CameraInfo info, Field field, VisionSystemSim visionSim) {
        super(info, field);
        var properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, org.wpilib.math.geometry.Rotation2d.fromDegrees(70));
        properties.setFPS(30);
        visionSim.addCamera(new PhotonCameraSim(camera(), properties, field), info.robotToCamera());
    }
}
