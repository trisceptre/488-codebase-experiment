package first.robot.vision;

import org.wpilib.math.geometry.Transform3d;

/**
 * This class is used to provide information about the cameras on the robot.
 */
public record CameraInfo(String name, String photonCameraName, Transform3d robotToCamera, double stdDevFactor) {}
