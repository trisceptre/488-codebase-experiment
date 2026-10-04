package frc.team488.mechanisms.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.LoopParticipant;
import frc.team488.Pose;
import frc.team488.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;

class DriveTest {
    private final Scheduler scheduler = Scheduler.getDefault();
    private final double[] axes = new double[6]; // leftX, leftY, leftTrigger, rightTrigger, rightX, rightY
    private final JoystickInputs sticks = new JoystickInputs(
            () -> axes[0], () -> axes[1], () -> axes[2], () -> axes[3], () -> axes[4], () -> axes[5]);
    private FakeDriveIO io;
    private Pose pose;
    private Drive drive;
    private List<LoopParticipant> loop;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        scheduler.cancelAll();
        setAlliance(AllianceStationID.BLUE_1);
        var config = DriveConstants.CONFIG;
        io = new FakeDriveIO(config);
        pose = new Pose(config.kinematics(), config.moduleCount());
        drive = new Drive(config, io, pose);
        loop = List.of(drive, pose);
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        drive.close();
        pose.close();
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
        assertTrue(pose.get().getX() > 0.5, "x " + pose.get().getX());
    }

    @Test
    void forwardStickDrivesTowardBlueWallOnRed() {
        setAlliance(AllianceStationID.RED_1);
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[1] = -1.0;
        loops(25);
        assertTrue(pose.get().getX() < -0.5, "x " + pose.get().getX());
    }

    @Test
    void rightStickTurnsToFaceIt() {
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[4] = -1.0; // stick left: face +90 degrees
        loops(100);
        assertEquals(90.0, pose.heading().getDegrees(), 3.0);
    }

    @Test
    void rightStickIgnoredWithoutGyro() {
        io.gyroConnected = false;
        SimTestUtil.schedule(scheduler, drive.joystickDrive(sticks, () -> false));
        axes[4] = -1.0;
        loops(60);
        assertEquals(0.0, pose.heading().getDegrees(), 1e-6);
    }

    @Test
    void aimAtReachesHeading() {
        SimTestUtil.schedule(scheduler, drive.aimAt(sticks, () -> new Translation2d(5.0, 5.0)));
        loops(100);
        assertTrue(drive.atHeading());
        assertEquals(45.0, pose.heading().getDegrees(), 3.0);
    }

    @Test
    void lockWheelsAndDisabledRequests() {
        SimTestUtil.schedule(scheduler, drive.lockWheels());
        loops(2);
        assertEquals(FakeDriveIO.Request.LOCK, io.last);
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
}
