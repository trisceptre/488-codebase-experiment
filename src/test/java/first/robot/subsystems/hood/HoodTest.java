package first.robot.subsystems.hood;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HoodTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private double servo = Double.NaN;
    private Hood hood;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        hood = new Hood(new HoodIO() {
            @Override
            public void setServoPosition(double position) {
                servo = position;
            }
        });
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        SimTestUtil.tearDown();
    }

    private void seconds(double s) {
        for (int i = 0; i < Math.round(s / SimTestUtil.LOOP_SECONDS); i++) {
            SimTestUtil.step(List.of(hood), scheduler::run);
        }
    }

    @Test
    void fullTravelTakesTheServoSweepTime() {
        SimTestUtil.schedule(scheduler, hood.hold(() -> 1.0));
        seconds(1.0);
        assertFalse(hood.atGoal());
        assertEquals(1.0 / 3.0, hood.estimatedRatio(), 0.03);
        assertEquals(Hood.SERVO_MAX, servo, 1e-9);
        seconds(2.5);
        assertTrue(hood.atGoal());
    }

    @Test
    void lowersByDefaultAndDoesNotMoveWhileDisabled() {
        SimTestUtil.schedule(scheduler, hood.hold(() -> 0.5));
        seconds(2.0);
        scheduler.cancelAll();
        SimTestUtil.setEnabled(false);
        seconds(1.0);
        assertEquals(Hood.SERVO_MIN, servo, 1e-9);
        assertEquals(0.5, hood.estimatedRatio(), 0.03);
    }

    @Test
    void goalsAreClampedToTheRange() {
        hood.setGoal(2.0);
        assertEquals(1.0, hood.goal());
    }
}
