package frc.team488;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.mechanisms.drive.DriveConstants;
import frc.team488.testing.SimTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

class PoseTest {
    private Pose pose;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = DriveConstants.CONFIG;
        pose = new Pose(config.kinematics(), config.moduleCount());
    }

    @AfterEach
    void tearDown() {
        pose.close();
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
    void odometrySamplesMoveThePose() {
        pose.addOdometrySample(0.00, Rotation2d.ZERO, straight(0.0));
        pose.addOdometrySample(0.01, Rotation2d.ZERO, straight(0.5));
        pose.addOdometrySample(0.02, Rotation2d.ZERO, straight(1.0));
        assertEquals(1.0, pose.get().getX(), 1e-6);
        assertEquals(0.0, pose.get().getY(), 1e-6);
    }

    @Test
    void trustedVisionPullsTheEstimate() {
        pose.addOdometrySample(0.00, Rotation2d.ZERO, straight(0.0));
        pose.addOdometrySample(0.02, Rotation2d.ZERO, straight(0.0));
        pose.addVisionMeasurement(new Pose2d(1.0, 0.0, Rotation2d.ZERO), 0.02, VecBuilder.fill(0.01, 0.01, 0.01));
        assertTrue(pose.get().getX() > 0.5, "x " + pose.get().getX());
    }

    @Test
    void resetHeadingKeepsTranslation() {
        pose.reset(new Pose2d(2.0, 3.0, Rotation2d.ZERO));
        pose.resetHeading(Rotation2d.k180deg);
        assertEquals(2.0, pose.get().getX(), 1e-9);
        assertEquals(180.0, Math.abs(pose.heading().getDegrees()), 1e-9);
    }
}
