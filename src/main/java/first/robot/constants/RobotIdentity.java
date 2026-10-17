package first.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Map;

public enum RobotIdentity {
    COMP_2026,
    CHASSIS_2025,
    SIM;

    private static final Map<String, RobotIdentity> BY_SERIAL = Map.of();

    public static RobotIdentity detect() {
        if (!RobotBase.isReal()) {
            return SIM;
        }
        return BY_SERIAL.getOrDefault(RobotController.getSerialNumber(), COMP_2026);
    }
}
