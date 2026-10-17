package first.robot.vision;

import first.robot.vision.VisionIO.PoseObservation;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

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

    public static Rejection check(PoseObservation o, double fieldLength, double fieldWidth, Settings s) {
        // No targets, so there's no way we should use this data.
        if (o.tagCount() < 1) {
            return Rejection.NO_TAGS;
        }
        // For a single target we need to be above reliability threshold and very close.
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
        // Two or more targets tends to be very reliable, but there's still a limit for distance
        if (o.tagCount() > 1 && o.averageTagDistance() > s.maxMultiTagDistanceMeters()) {
            return Rejection.MULTI_TAG_TOO_FAR;
        }
        return Rejection.NONE;
    }

    public static Matrix<N3, N1> stdDevs(PoseObservation o, double cameraFactor, Settings s) {
        double scale = o.averageTagDistance() * o.averageTagDistance() / o.tagCount() * cameraFactor;
        double linear = s.linearStdDevBaselineMeters() * scale;
        double angular = s.angularStdDevBaselineRad() * scale;
        return VecBuilder.fill(linear, linear, angular);
    }
}
