package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class LoggedTunableNumberTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void returnsDefaultBeforeAnyEdit() {
        var number = new LoggedTunableNumber("Test/Default", 4.5);
        assertEquals(4.5, number.get());
        assertEquals(4.5, number.getAsDouble());
    }

    @Test
    void importantIsSavedOnTheRobotAndDebugIsNot() {
        new LoggedTunableNumber("Test/Important", 1.0);
        new LoggedTunableNumber("Test/Debug", 1.0, LoggedTunableNumber.Level.DEBUG);
        var nt = NetworkTableInstance.getDefault();
        assertTrue(nt.getTopic("/Preferences/Test/Important").isPersistent());
        assertFalse(nt.getTopic("/Tuning/Test/Debug").isPersistent());
    }

    @Test
    void savedValueWinsOverCodeDefault() {
        var entry = NetworkTableInstance.getDefault().getEntry("/Preferences/Test/Saved");
        entry.setDouble(9.0);
        new LoggedTunableNumber("Test/Saved", 1.0);
        assertEquals(9.0, entry.getDouble(0.0));
    }

    @Test
    void hasChangedIsTrueOnlyOncePerIdForAnUnchangedValue() {
        var number = new LoggedTunableNumber("Test/Changed", 1.0);
        assertTrue(number.hasChanged(1));
        assertFalse(number.hasChanged(1));
        assertTrue(number.hasChanged(2));
    }
}
