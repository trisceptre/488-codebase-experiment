package first.robot.constants;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

public enum RobotIdentity {
    COMP_2026,
    CHASSIS_2025,
    SIM;

    public static final String CONTRACT_KEY = "ContractToUse";
    private static final String COMPETITION = "Competition";

    public static RobotIdentity detect() {
        if (!RobotBase.isReal()) {
            return SIM;
        }
        // Initialize the contract to use if this is a fresh robot. Assume competition since that's the safest.
        if (!Preferences.containsKey(CONTRACT_KEY)) {
            Preferences.setString(CONTRACT_KEY, COMPETITION);
        }
        return fromContract(Preferences.getString(CONTRACT_KEY, COMPETITION));
    }

    static RobotIdentity fromContract(String contract) {
        return "2025".equals(contract) ? CHASSIS_2025 : COMP_2026;
    }
}
