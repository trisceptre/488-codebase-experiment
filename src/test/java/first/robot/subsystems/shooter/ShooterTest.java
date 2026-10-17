package first.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.constants.Hardware2026;
import first.robot.testing.SimTestUtil;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.MotorIOSim;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private List<MotorIOSim> ios;
    private Shooter shooter;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        ios = Hardware2026.SHOOTER.stream().map(MotorIOSim::new).toList();
        shooter = new Shooter(Hardware2026.SHOOTER, List.<MotorIO>copyOf(ios));
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        shooter.close();
        SimTestUtil.tearDown();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            SimTestUtil.step(List.of(shooter), scheduler::run);
        }
    }

    @Test
    void portedGainsSpinUpToAPresetAndReportReady() {
        SimTestUtil.schedule(scheduler, shooter.spinUp(() -> RPM.of(3800)));
        loops(5);
        assertFalse(shooter.isReadyToFire());
        loops(150);
        assertTrue(shooter.isReadyToFire(), "rpm " + shooter.velocityRpm());
        assertEquals(3800, shooter.velocityRpm(), 300);
    }

    @Test
    void releasingTheButtonStopsAndIsNeverReady() {
        var spin = shooter.spinUp(() -> RPM.of(3000));
        SimTestUtil.schedule(scheduler, spin);
        loops(100);
        scheduler.cancel(spin);
        loops(2);
        assertEquals(0.0, shooter.goalRpm());
        assertFalse(shooter.isReadyToFire());
    }

    @Test
    void trimShiftsTheGoalWithoutInterruptingSpinUp() {
        var spin = shooter.spinUp(() -> RPM.of(3000));
        SimTestUtil.schedule(scheduler, spin);
        loops(1);
        double before = shooter.trimmedGoalRpm() - 3000;
        scheduler.schedule(shooter.trimUp());
        loops(2);
        assertTrue(scheduler.isScheduled(spin));
        assertEquals(before + 15.0, shooter.trimmedGoalRpm() - 3000, 1e-9);
        scheduler.schedule(shooter.trimDown());
        loops(1);
    }

    @Test
    void disconnectedMotorIsDroppedAndLowPowerRunsOne() {
        SimTestUtil.schedule(scheduler, shooter.spinUp(() -> RPM.of(3000)));
        ios.get(0).setDisconnected(true);
        loops(150);
        assertTrue(shooter.isReadyToFire(), "the two healthy motors still reach the goal");

        scheduler.schedule(shooter.setLowPowerMode(true));
        ios.get(1).setDisconnected(true);
        loops(200);
        assertTrue(shooter.isReadyToFire(), "rpm " + shooter.velocityRpm());
    }
}
