package first.robot.mechanisms.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.testing.SimTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.SimHooks;

class HeadingModuleTest {
    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.tearDown();
    }

    @Test
    void turnsTheShortWayAcrossPi() {
        var heading = new HeadingModule(5.0, 0.0, 10.0);
        double omega = heading.calculate(Rotation2d.fromDegrees(170), Rotation2d.fromDegrees(-170));
        assertTrue(omega > 0, "omega " + omega);
    }

    @Test
    void outputIsClamped() {
        var heading = new HeadingModule(100.0, 0.0, 3.0);
        assertEquals(3.0, heading.calculate(Rotation2d.ZERO, Rotation2d.fromDegrees(90)), 1e-9);
    }

    @Test
    void atSetpointNeedsToleranceForDebounce() {
        var heading = new HeadingModule(5.0, 0.0, 10.0);
        heading.calculate(Rotation2d.fromDegrees(1), Rotation2d.ZERO);
        assertFalse(heading.atSetpoint(), "not yet debounced");
        for (int i = 0; i < 6; i++) {
            SimHooks.stepTiming(0.02);
            heading.calculate(Rotation2d.fromDegrees(1), Rotation2d.ZERO);
        }
        assertTrue(heading.atSetpoint());
        heading.calculate(Rotation2d.fromDegrees(10), Rotation2d.ZERO);
        assertFalse(heading.atSetpoint());
    }
}
