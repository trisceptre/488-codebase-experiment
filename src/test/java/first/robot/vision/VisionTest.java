package first.robot.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import first.robot.RobotState;
import first.robot.constants.RobotConfigs;
import first.robot.testing.SimTestUtil;
import first.robot.vision.VisionIO.PoseObservation;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionTest {
    private static final class FakeCamera implements VisionIO {
        PoseObservation[] next = new PoseObservation[0];

        @Override
        public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.poseObservations = next;
            next = new PoseObservation[0];
        }
    }

    private RobotState robotState;
    private Vision vision;
    private FakeCamera camera;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = RobotConfigs.COMP_2026.drive();
        robotState = new RobotState(config.kinematics(), config.moduleCount());
        camera = new FakeCamera();
        var info = new CameraInfo("Test", "TestCam", new Transform3d(), 1.0);
        vision = new Vision(
                List.of(info),
                List.of(camera),
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                robotState);
    }

    @AfterEach
    void tearDown() {
        vision.close();
        robotState.close();
        SimTestUtil.tearDown();
    }

    @Test
    void acceptedObservationMovesPoseRejectedDoesNot() {
        double now = Timer.getTimestamp();
        var still = new SwerveModulePosition[4];
        Arrays.fill(still, new SwerveModulePosition());
        robotState.addOdometryObservation(new RobotState.OdometryObservation(now, still, Rotation2d.kZero));
        camera.next = new PoseObservation[] {
            new PoseObservation(now, new Pose3d(4.0, 3.0, 0.0, new Rotation3d()), 0.0, 2, 1.0)
        };
        vision.updateInputs();
        assertTrue(
                robotState.getEstimatedPose().getX() > 0.5,
                "accepted: x " + robotState.getEstimatedPose().getX());

        double before = robotState.getEstimatedPose().getY();
        camera.next = new PoseObservation[] {
            new PoseObservation(now, new Pose3d(4.0, 7.0, 2.0, new Rotation3d()), 0.0, 2, 1.0)
        };
        vision.updateInputs();
        assertTrue(
                Math.abs(robotState.getEstimatedPose().getY() - before) < 1e-9,
                "rejected observation moved robotState");
    }
}
