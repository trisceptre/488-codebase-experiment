package frc.team488.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.Pose;
import frc.team488.mechanisms.drive.DriveConstants;
import frc.team488.testing.SimTestUtil;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.system.Timer;

class VisionTest {
    private static final class FakeCamera implements AprilTagVisionIO {
        VisionPoseObservation[] next = new VisionPoseObservation[0];

        @Override
        public void updateInputs(AprilTagVisionIOInputs inputs) {
            inputs.connected = true;
            inputs.observations = next;
            next = new VisionPoseObservation[0];
        }
    }

    private Pose pose;
    private Vision vision;
    private FakeCamera camera;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = DriveConstants.CONFIG;
        pose = new Pose(config.kinematics(), config.moduleCount());
        camera = new FakeCamera();
        var info = new CameraInfo("Test", "TestCam", new Transform3d(), 1.0);
        vision = new Vision(List.of(info), List.of(camera), Field.loadField(Fields.FRC_2026_REBUILT_WELDED), pose);
    }

    @AfterEach
    void tearDown() {
        vision.close();
        pose.close();
        SimTestUtil.tearDown();
    }

    @Test
    void acceptedObservationMovesPoseRejectedDoesNot() {
        double now = Timer.getTimestamp();
        // The estimator ignores vision until it has odometry history to place it in.
        var still = new SwerveModulePosition[4];
        Arrays.fill(still, new SwerveModulePosition());
        pose.addOdometrySample(now, Rotation2d.ZERO, still);
        camera.next = new VisionPoseObservation[] {
            new VisionPoseObservation(now, new Pose3d(4.0, 3.0, 0.0, new Rotation3d()), 0.0, 2, 1.0)
        };
        vision.updateInputs();
        assertTrue(pose.get().getX() > 0.5, "accepted: x " + pose.get().getX());

        double before = pose.get().getY();
        camera.next = new VisionPoseObservation[] {
            new VisionPoseObservation(now, new Pose3d(4.0, 7.0, 2.0, new Rotation3d()), 0.0, 2, 1.0) // z too high
        };
        vision.updateInputs();
        assertTrue(Math.abs(pose.get().getY() - before) < 1e-9, "rejected observation moved pose");
    }
}
