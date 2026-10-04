package frc.team488.config;

import java.util.Map;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;

/** Which physical robot (or simulation) this code is running on. */
public enum RobotIdentity {
    COMP,
    SIM;

    // TODO(season): map each robot controller's serial number to its identity.
    private static final Map<String, RobotIdentity> BY_SERIAL = Map.of();

    public static RobotIdentity detect() {
        if (!RobotBase.isReal()) {
            return SIM;
        }
        return BY_SERIAL.getOrDefault(RobotController.getSerialNumber(), COMP);
    }
}
