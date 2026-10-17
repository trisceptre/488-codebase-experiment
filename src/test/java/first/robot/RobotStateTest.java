package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import first.robot.constants.RobotConfigs;
import first.robot.testing.SimTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotStateTest {
    private RobotState robotState;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = RobotConfigs.COMP_2026.drive();
        robotState = new RobotState(config.kinematics(), config.moduleCount());
    }

    @AfterEach
    void tearDown() {
        robotState.close();
        SimTestUtil.tearDown();
    }

    private static SwerveModulePosition[] straight(double meters) {
        var p = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            p[i] = new SwerveModulePosition(meters, Rotation2d.kZero);
        }
        return p;
    }

    @Test
    void odometryObservationsMoveThePose() {
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.00, straight(0.0), Rotation2d.kZero));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.01, straight(0.5), Rotation2d.kZero));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.02, straight(1.0), Rotation2d.kZero));
        assertEquals(1.0, robotState.getEstimatedPose().getX(), 1e-6);
        assertEquals(0.0, robotState.getEstimatedPose().getY(), 1e-6);
    }

    @Test
    void trustedVisionPullsTheEstimate() {
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.00, straight(0.0), Rotation2d.kZero));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.02, straight(0.0), Rotation2d.kZero));
        robotState.addVisionObservation(new RobotState.VisionObservation(
                0.02, new Pose3d(new Pose2d(1.0, 0.0, Rotation2d.kZero)), VecBuilder.fill(0.01, 0.01, 0.01)));
        assertTrue(
                robotState.getEstimatedPose().getX() > 0.5,
                "x " + robotState.getEstimatedPose().getX());
    }

    @Test
    void resetRotationKeepsTranslation() {
        robotState.resetPose(new Pose2d(2.0, 3.0, Rotation2d.kZero));
        robotState.resetRotation(Rotation2d.k180deg);
        assertEquals(2.0, robotState.getEstimatedPose().getX(), 1e-9);
        assertEquals(180.0, Math.abs(robotState.getRotation().getDegrees()), 1e-9);
    }
}
