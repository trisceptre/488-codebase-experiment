package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.LoopParticipant;
import first.robot.RobotState;
import first.robot.constants.RobotConfigs;
import first.robot.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final double[] axes = new double[6];
    private final JoystickInputs sticks = new JoystickInputs(
            () -> axes[0], () -> axes[1], () -> axes[2], () -> axes[3], () -> axes[4], () -> axes[5]);
    private FakeDriveIO io;
    private RobotState robotState;
    private Drive drive;
    private List<LoopParticipant> loop;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        setAlliance(AllianceStationID.Blue1);
        var config = RobotConfigs.COMP_2026.drive();
        io = new FakeDriveIO(config);
        robotState = new RobotState(config.kinematics(), config.moduleCount());
        drive = new Drive(config, io, robotState);
        loop = List.of(drive, robotState);
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        drive.close();
        robotState.close();
        SimTestUtil.tearDown();
    }

    private static void setAlliance(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            SimTestUtil.step(loop, scheduler::run);
        }
    }

    @Test
    void forwardStickDrivesAwayFromBlueWall() {
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[1] = -1.0;
        loops(25);
        assertTrue(
                robotState.getEstimatedPose().getX() > 0.5,
                "x " + robotState.getEstimatedPose().getX());
    }

    @Test
    void forwardStickDrivesTowardBlueWallOnRed() {
        setAlliance(AllianceStationID.Red1);
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[1] = -1.0;
        loops(25);
        assertTrue(
                robotState.getEstimatedPose().getX() < -0.5,
                "x " + robotState.getEstimatedPose().getX());
    }

    @Test
    void rightStickTurnsToFaceIt() {
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[4] = -1.0;
        loops(100);
        assertEquals(90.0, robotState.getRotation().getDegrees(), 3.0);
    }

    @Test
    void rightStickIgnoredWithoutGyro() {
        io.gyroConnected = false;
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[4] = -1.0;
        loops(60);
        assertEquals(0.0, robotState.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void joystickDriveAtAngleReachesHeading() {
        SimTestUtil.schedule(
                scheduler, drive.joystickDriveAtAngle(sticks, () -> Rotation2d.fromDegrees(45), () -> true));
        loops(100);
        assertTrue(drive.atHeading());
        assertEquals(45.0, robotState.getRotation().getDegrees(), 3.0);
    }

    @Test
    void joystickDriveAtAngleUsesSticksWhenNotAiming() {
        SimTestUtil.schedule(
                scheduler, drive.joystickDriveAtAngle(sticks, () -> Rotation2d.fromDegrees(45), () -> false));
        loops(100);
        assertEquals(0.0, robotState.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void fullStickDrivesAtMaxTargetSpeed() {
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[1] = -1.0;
        loops(2);
        assertEquals(4.5, io.robotVelocity.vxMetersPerSecond, 1e-9);
    }

    @Test
    void fullTriggerTurnsAtMaxTargetTurnRate() {
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[2] = 1.0;
        loops(2);
        assertEquals(8.0, io.robotVelocity.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void wheelSpeedsStayUnderMaxTargetSpeed() {
        SimTestUtil.schedule(scheduler, drive.run(() -> drive.runRobotVelocity(new ChassisSpeeds(4.5, 0.0, 8.0))));
        loops(2);
        double fastest = 0.0;
        for (var state : RobotConfigs.COMP_2026.drive().kinematics().toSwerveModuleStates(io.robotVelocity)) {
            fastest = Math.max(fastest, Math.abs(state.speedMetersPerSecond));
        }
        assertEquals(4.5, fastest, 1e-9);
        assertEquals(8.0 / 4.5, io.robotVelocity.omegaRadiansPerSecond / io.robotVelocity.vxMetersPerSecond, 1e-9);
    }

    @Test
    void stopWithXAndDisabledRequests() {
        SimTestUtil.schedule(scheduler, drive.stopWithX());
        loops(2);
        assertEquals(FakeDriveIO.Request.X, io.last);
        SimTestUtil.setEnabled(false);
        loops(1);
        assertEquals(FakeDriveIO.Request.IDLE, io.last);
    }

    @Test
    void idleByDefault() {
        loops(2);
        assertEquals(FakeDriveIO.Request.IDLE, io.last);
        assertFalse(drive.atHeading());
    }

    @Test
    void wheelRadiusCharacterizationRecoversConfiguredRadius() {
        SimTestUtil.schedule(scheduler, drive.wheelRadiusCharacterization());
        loops(150);
        assertEquals(RobotConfigs.COMP_2026.drive().wheelRadiusMeters(), drive.getWheelRadiusEstimateMeters(), 1e-3);
    }
}
