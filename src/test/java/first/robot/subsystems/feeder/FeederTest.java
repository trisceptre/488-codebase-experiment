package first.robot.subsystems.feeder;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.constants.Hardware2026;
import first.robot.testing.SimTestUtil;
import first.robot.util.motor.MotorIO;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FeederTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final double[] amps = {0.0};
    private Feeder feeder;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        feeder = new Feeder(Hardware2026.FEEDER, new MotorIO() {
            @Override
            public void updateInputs(MotorIOInputs inputs) {
                inputs.connected = true;
                inputs.statorAmps = amps[0];
            }
        });
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        feeder.close();
        SimTestUtil.tearDown();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            SimTestUtil.step(List.of(feeder), scheduler::run);
        }
    }

    @Test
    void shootingFinishesOnlyAfterAShotAndTheSettleTime() {
        var wait = feeder.waitUntilShootingFinished();
        SimTestUtil.schedule(scheduler, wait);
        loops(50);
        assertTrue(scheduler.isScheduled(wait), "no shot seen yet");
        amps[0] = 20.0;
        loops(5);
        amps[0] = 0.0;
        loops(10);
        assertTrue(scheduler.isScheduled(wait), "still settling");
        loops(20);
        assertFalse(scheduler.isScheduled(wait));
    }

    @Test
    void firingIsReportedForTheSimulation() {
        var fire = feeder.fire();
        SimTestUtil.schedule(scheduler, fire);
        loops(2);
        assertTrue(feeder.isFiring());
        scheduler.cancel(fire);
        loops(2);
        assertFalse(feeder.isFiring());
    }
}
