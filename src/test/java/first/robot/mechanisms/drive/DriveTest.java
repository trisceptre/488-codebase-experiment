package first.robot.mechanisms.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.LoopParticipant;
import first.robot.RobotState;
import first.robot.constants.RobotConfigs;
import first.robot.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;

class DriveTest {
    private final Scheduler scheduler = Scheduler.getDefault();
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
        scheduler.cancelAll();
        setAlliance(AllianceStationID.BLUE_1);
        var config = RobotConfigs.COMP.drive();
        io = new FakeDriveIO(config);
        robotState = new RobotState(config.kinematics(), config.moduleCount());
        drive = new Drive(config, io, robotState);
        loop = List.of(drive, robotState);
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
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
        setAlliance(AllianceStationID.RED_1);
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
        SimTestUtil.schedule(scheduler, drive.joystickDriveAtAngle(sticks, () -> Rotation2d.fromDegrees(45)));
        loops(100);
        assertTrue(drive.atHeading());
        assertEquals(45.0, robotState.getRotation().getDegrees(), 3.0);
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
        assertEquals(RobotConfigs.COMP.drive().wheelRadiusMeters(), drive.getWheelRadiusEstimateMeters(), 1e-3);
    }
}
