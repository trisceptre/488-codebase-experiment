package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;

class LoggedTunableBooleanTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize());
    }

    @Test
    void returnsDefaultBeforeAnyEdit() {
        assertTrue(new LoggedTunableBoolean("Test/BooleanDefault", true).get());
        assertFalse(new LoggedTunableBoolean("Test/BooleanDefaultFalse", false).getAsBoolean());
    }

    @Test
    void isSavedOnTheRobot() {
        new LoggedTunableBoolean("Test/BooleanImportant", true);
        assertTrue(NetworkTableInstance.getDefault()
                .getTopic("/Preferences/Test/BooleanImportant")
                .isPersistent());
    }

    @Test
    void savedValueWinsOverCodeDefault() {
        var entry = NetworkTableInstance.getDefault().getEntry("/Preferences/Test/BooleanSaved");
        entry.setBoolean(false);
        new LoggedTunableBoolean("Test/BooleanSaved", true);
        assertFalse(entry.getBoolean(true));
    }
}
