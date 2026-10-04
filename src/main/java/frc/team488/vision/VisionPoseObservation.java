package frc.team488.vision;

import org.wpilib.math.geometry.Pose3d;

/**
 * One robot pose estimated from AprilTags by one camera.
 *
 * @param timestamp capture time in WPILib timer seconds
 * @param ambiguity Photon pose ambiguity (single-tag only; 0 for multi-tag)
 * @param averageTagDistance mean camera-to-tag distance in meters
 */
public record VisionPoseObservation(
        double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}
