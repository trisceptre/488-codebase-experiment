package frc.team488.vision;

import org.wpilib.math.geometry.Transform3d;

/**
 * Where a camera is mounted and how much to trust it.
 *
 * @param name log name, e.g. "Left"
 * @param photonCameraName the camera's name in the PhotonVision UI
 * @param stdDevFactor multiplies this camera's std devs; above 1 trusts it less
 */
public record CameraInfo(String name, String photonCameraName, Transform3d robotToCamera, double stdDevFactor) {}
