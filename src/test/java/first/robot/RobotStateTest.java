package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.MetersPerSecond;

import first.robot.generated.TunerConstants;
import first.robot.mechanisms.drive.SwerveConfig;
import first.robot.testing.SimTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

class RobotStateTest {
    private RobotState robotState;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = SwerveConfig.create(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
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
            p[i] = new SwerveModulePosition(meters, Rotation2d.ZERO);
        }
        return p;
    }

    @Test
    void odometryObservationsMoveThePose() {
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.00, straight(0.0), Rotation2d.ZERO));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.01, straight(0.5), Rotation2d.ZERO));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.02, straight(1.0), Rotation2d.ZERO));
        assertEquals(1.0, robotState.getEstimatedPose().getX(), 1e-6);
        assertEquals(0.0, robotState.getEstimatedPose().getY(), 1e-6);
    }

    @Test
    void trustedVisionPullsTheEstimate() {
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.00, straight(0.0), Rotation2d.ZERO));
        robotState.addOdometryObservation(new RobotState.OdometryObservation(0.02, straight(0.0), Rotation2d.ZERO));
        robotState.addVisionObservation(new RobotState.VisionObservation(
                0.02, new Pose3d(new Pose2d(1.0, 0.0, Rotation2d.ZERO)), VecBuilder.fill(0.01, 0.01, 0.01)));
        assertTrue(
                robotState.getEstimatedPose().getX() > 0.5,
                "x " + robotState.getEstimatedPose().getX());
    }

    @Test
    void resetRotationKeepsTranslation() {
        robotState.resetPose(new Pose2d(2.0, 3.0, Rotation2d.ZERO));
        robotState.resetRotation(Rotation2d.k180deg);
        assertEquals(2.0, robotState.getEstimatedPose().getX(), 1e-9);
        assertEquals(180.0, Math.abs(robotState.getRotation().getDegrees()), 1e-9);
    }
}
