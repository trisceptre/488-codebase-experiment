package frc.team488.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.vision.VisionFilter.Rejection;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

class VisionFilterTest {
    private static final VisionFilter.Settings S = VisionFilter.Settings.DEFAULTS;
    private static final double LENGTH = 16.5;
    private static final double WIDTH = 8.1;

    private static VisionPoseObservation obs(double x, double y, double z, double ambiguity, int tags, double dist) {
        return new VisionPoseObservation(1.0, new Pose3d(x, y, z, Rotation3d.ZERO), ambiguity, tags, dist);
    }

    private static Rejection check(VisionPoseObservation o) {
        return VisionFilter.check(o, LENGTH, WIDTH, S);
    }

    @Test
    void acceptsAGoodMultiTagObservation() {
        assertEquals(Rejection.NONE, check(obs(5, 4, 0, 0.0, 2, 3.0)));
    }

    @Test
    void rejectsEachRule() {
        assertEquals(Rejection.NO_TAGS, check(obs(5, 4, 0, 0.0, 0, 3.0)));
        assertEquals(Rejection.AMBIGUITY, check(obs(5, 4, 0, 0.31, 1, 0.8)));
        assertEquals(Rejection.Z_ERROR, check(obs(5, 4, 0.76, 0.0, 2, 3.0)));
        assertEquals(Rejection.OUTSIDE_FIELD, check(obs(-0.1, 4, 0, 0.0, 2, 3.0)));
        assertEquals(Rejection.OUTSIDE_FIELD, check(obs(5, 8.2, 0, 0.0, 2, 3.0)));
        assertEquals(Rejection.TOO_CLOSE, check(obs(5, 4, 0, 0.0, 2, 0.4)));
        assertEquals(Rejection.SINGLE_TAG_TOO_FAR, check(obs(5, 4, 0, 0.1, 1, 1.1)));
        assertEquals(Rejection.MULTI_TAG_TOO_FAR, check(obs(5, 4, 0, 0.0, 3, 5.1)));
    }

    @Test
    void stdDevsGrowWithDistanceAndShrinkWithTags() {
        double near = VisionFilter.stdDevs(obs(5, 4, 0, 0, 1, 1.0), 1.0, S).get(0, 0);
        double far = VisionFilter.stdDevs(obs(5, 4, 0, 0, 1, 2.0), 1.0, S).get(0, 0);
        double farTwoTags =
                VisionFilter.stdDevs(obs(5, 4, 0, 0, 2, 2.0), 1.0, S).get(0, 0);
        assertEquals(0.02, near, 1e-9);
        assertEquals(4 * near, far, 1e-9);
        assertEquals(far / 2, farTwoTags, 1e-9);
        double angular = VisionFilter.stdDevs(obs(5, 4, 0, 0, 1, 1.0), 2.0, S).get(2, 0);
        assertEquals(0.12, angular, 1e-9);
        assertTrue(VisionFilter.stdDevs(obs(5, 4, 0, 0, 1, 1.0), 3.0, S).get(1, 0) > near);
    }
}
