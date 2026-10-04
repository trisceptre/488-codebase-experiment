package frc.team488.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

class TunableNumberTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize());
    }

    @Test
    void returnsDefaultWhenNotTuning() {
        var number = new TunableNumber("Test/NotTuning", 4.5, false);
        assertEquals(4.5, number.get());
        assertEquals(4.5, number.getAsDouble());
    }

    @Test
    void returnsDefaultWhenTuningAndUntouched() {
        var number = new TunableNumber("Test/Tuning", 2.0, true);
        assertEquals(2.0, number.get());
    }

    @Test
    void hasChangedIsTrueOnlyOncePerIdForAnUnchangedValue() {
        var number = new TunableNumber("Test/Changed", 1.0, false);
        assertTrue(number.hasChanged(1));
        assertFalse(number.hasChanged(1));
        assertTrue(number.hasChanged(2));
    }

    @Test
    void tuningModeIsOffByDefault() {
        assertFalse(TunableNumber.TUNING_MODE);
    }
}
