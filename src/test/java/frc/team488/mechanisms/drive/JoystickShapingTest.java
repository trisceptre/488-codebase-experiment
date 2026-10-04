package frc.team488.mechanisms.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class JoystickShapingTest {
    private static final JoystickShaping.Settings S = JoystickShaping.Settings.DEFAULTS;

    private static JoystickShaping.Intent shape(
            double lx, double ly, double lt, double rt, double rx, double ry, boolean red, boolean precision) {
        return JoystickShaping.shape(lx, ly, lt, rt, rx, ry, red, precision, S);
    }

    @Test
    void deadbandZeroesSmallInputsAndRescales() {
        assertEquals(0.0, JoystickShaping.deadband(0.1, 0.15));
        assertEquals(1.0, JoystickShaping.deadband(1.0, 0.15), 1e-9);
        assertEquals(-1.0, JoystickShaping.deadband(-1.0, 0.15), 1e-9);
        assertEquals(0.5, JoystickShaping.deadband(0.575, 0.15), 1e-9);
    }

    @Test
    void fullForwardStickIsPlusXOnBlueAndMinusXOnRed() {
        assertEquals(1.0, shape(0, -1, 0, 0, 0, 0, false, false).vxFraction(), 1e-9);
        assertEquals(-1.0, shape(0, -1, 0, 0, 0, 0, true, false).vxFraction(), 1e-9);
        assertEquals(1.0, shape(-1, 0, 0, 0, 0, 0, false, false).vyFraction(), 1e-9);
    }

    @Test
    void translationIsSquared() {
        double half = 0.15 + 0.5 * 0.85; // deadbands to 0.5
        assertEquals(0.25, shape(0, -half, 0, 0, 0, 0, false, false).vxFraction(), 1e-9);
    }

    @Test
    void triggersRotateLeftTriggerCounterClockwise() {
        assertEquals(1.0, shape(0, 0, 1, 0, 0, 0, false, false).omegaFraction(), 1e-9);
        assertEquals(-1.0, shape(0, 0, 0, 1, 0, 0, false, false).omegaFraction(), 1e-9);
    }

    @Test
    void precisionScalesTranslationAndRotation() {
        var intent = shape(0, -1, 1, 0, 0, 0, false, true);
        assertEquals(0.1, intent.vxFraction(), 1e-9);
        assertEquals(0.2, intent.omegaFraction(), 1e-9);
    }

    @Test
    void rightStickSetsHeadingPastThreshold() {
        assertTrue(shape(0, 0, 0, 0, 0.3, 0, false, false).heading().isEmpty());
        // Stick pointed left (rightX = -1) faces +90 degrees on blue.
        assertEquals(
                90.0,
                shape(0, 0, 0, 0, -1, 0, false, false).heading().orElseThrow().getDegrees(),
                1e-9);
        // Same stick on red faces -90 degrees (rotated 180).
        assertEquals(
                -90.0,
                shape(0, 0, 0, 0, -1, 0, true, false).heading().orElseThrow().getDegrees(),
                1e-9);
    }
}
