package frc.team488.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/** Decides whether to fuse a vision observation and how much to trust it. */
public final class VisionFilter {
    private VisionFilter() {}

    public record Settings(
            double maxAmbiguity,
            double maxZErrorMeters,
            double minTagDistanceMeters,
            double maxSingleTagDistanceMeters,
            double maxMultiTagDistanceMeters,
            double linearStdDevBaselineMeters,
            double angularStdDevBaselineRad) {
        /** 488's 2026 values. */
        public static final Settings DEFAULTS = new Settings(0.3, 0.75, 0.5, 1.0, 5.0, 0.02, 0.06);
    }

    public enum Rejection {
        NONE,
        NO_TAGS,
        AMBIGUITY,
        Z_ERROR,
        OUTSIDE_FIELD,
        TOO_CLOSE,
        SINGLE_TAG_TOO_FAR,
        MULTI_TAG_TOO_FAR
    }

    public static Rejection check(VisionPoseObservation o, double fieldLength, double fieldWidth, Settings s) {
        if (o.tagCount() < 1) {
            return Rejection.NO_TAGS;
        }
        if (o.tagCount() == 1 && o.ambiguity() > s.maxAmbiguity()) {
            return Rejection.AMBIGUITY;
        }
        if (Math.abs(o.pose().getZ()) > s.maxZErrorMeters()) {
            return Rejection.Z_ERROR;
        }
        double x = o.pose().getX();
        double y = o.pose().getY();
        if (x < 0.0 || x > fieldLength || y < 0.0 || y > fieldWidth) {
            return Rejection.OUTSIDE_FIELD;
        }
        if (o.averageTagDistance() < s.minTagDistanceMeters()) {
            return Rejection.TOO_CLOSE;
        }
        if (o.tagCount() == 1 && o.averageTagDistance() > s.maxSingleTagDistanceMeters()) {
            return Rejection.SINGLE_TAG_TOO_FAR;
        }
        if (o.tagCount() > 1 && o.averageTagDistance() > s.maxMultiTagDistanceMeters()) {
            return Rejection.MULTI_TAG_TOO_FAR;
        }
        return Rejection.NONE;
    }

    /** [x, y, theta] std devs: baseline × distance² ÷ tag count × camera factor. */
    public static Matrix<N3, N1> stdDevs(VisionPoseObservation o, double cameraFactor, Settings s) {
        double scale = o.averageTagDistance() * o.averageTagDistance() / o.tagCount() * cameraFactor;
        double linear = s.linearStdDevBaselineMeters() * scale;
        double angular = s.angularStdDevBaselineRad() * scale;
        return VecBuilder.fill(linear, linear, angular);
    }
}
