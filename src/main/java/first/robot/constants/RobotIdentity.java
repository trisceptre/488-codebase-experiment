package first.robot.constants;

import java.util.Map;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;

public enum RobotIdentity {
    COMP,
    SIM;

    private static final Map<String, RobotIdentity> BY_SERIAL = Map.of();

    public static RobotIdentity detect() {
        if (!RobotBase.isReal()) {
            return SIM;
        }
        return BY_SERIAL.getOrDefault(RobotController.getSerialNumber(), COMP);
    }
}
