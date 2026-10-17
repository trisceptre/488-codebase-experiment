package first.robot.util.motor;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import first.robot.testing.SimTestUtil;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VelocityControlTest {
    private MotorIOSim io;
    private VelocityControl flywheel;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        var config = new MotorConfig(
                "VelocityTest",
                new CANId(1),
                List.of(),
                false,
                false,
                1.0,
                new MotorConfig.Gains(0.0, 0.12, 0.0, 0.0, 0.3, 0.0, MotorConfig.GravityType.NONE),
                new MotorConfig.CurrentLimits(60, 120),
                Optional.empty(),
                MotorConfig.Shaping.NONE,
                new MotorConfig.SimModel.Flywheel(DCMotor.getKrakenX60(1), 0.004));
        io = new MotorIOSim(config);
        flywheel = new VelocityControl(config, io, RPM.of(50));
    }

    @AfterEach
    void tearDown() {
        flywheel.close();
        SimTestUtil.tearDown();
    }

    private void loops(int count) {
        for (int i = 0; i < count; i++) {
            SimTestUtil.step(List.of(flywheel));
        }
    }

    @Test
    void reachesGoalAndReportsAtGoal() {
        flywheel.setGoal(RPM.of(3000));
        loops(1);
        assertFalse(flywheel.atGoal(), "must not be at goal immediately");
        loops(100);
        assertTrue(flywheel.atGoal());
        assertEquals(3000, flywheel.velocity().in(RPM), 50);
    }

    @Test
    void changingGoalClearsAtGoal() {
        flywheel.setGoal(RPM.of(3000));
        loops(100);
        assertTrue(flywheel.atGoal());
        flywheel.setGoal(RPM.of(1000));
        assertFalse(flywheel.atGoal());
    }

    @Test
    void stopClearsGoalAndCoasts() {
        flywheel.setGoal(RPM.of(3000));
        loops(50);
        flywheel.stop();
        loops(1);
        assertTrue(flywheel.goal().isEmpty());
        assertFalse(flywheel.atGoal());
    }

    @Test
    void openLoopVoltageSpinsWithoutReachingGoalAndAGoalCancelsIt() {
        flywheel.runVoltage(6.0);
        loops(100);
        assertTrue(
                flywheel.velocity().in(RPM) > 1000, "rpm " + flywheel.velocity().in(RPM));
        assertTrue(flywheel.goal().isEmpty());
        assertFalse(flywheel.atGoal());
        assertTrue(flywheel.statorAmps() >= 0.0);

        flywheel.setGoal(RPM.of(1500));
        loops(100);
        assertTrue(flywheel.atGoal());
        assertEquals(1500, flywheel.velocity().in(RPM), 50);
    }

    @Test
    void disabledRobotSendsNeutral() {
        flywheel.setGoal(RPM.of(3000));
        SimTestUtil.setEnabled(false);
        loops(50);
        assertEquals(0.0, flywheel.velocity().in(RPM), 1.0);
    }

    @Test
    void disconnectIsDebouncedAndBlocksAtGoal() {
        flywheel.setGoal(RPM.of(3000));
        loops(100);
        assertTrue(flywheel.atGoal());
        io.setDisconnected(true);
        loops(5);
        assertTrue(flywheel.isConnected());
        loops(30);
        assertFalse(flywheel.isConnected());
        assertFalse(flywheel.atGoal());
    }
}
