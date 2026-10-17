package first.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class RobotIdentityTest {
    @Test
    void contractToUsePicksTheRobot() {
        assertEquals(RobotIdentity.CHASSIS_2025, RobotIdentity.fromContract("2025"));
        assertEquals(RobotIdentity.COMP_2026, RobotIdentity.fromContract("Competition"));
        assertEquals(RobotIdentity.COMP_2026, RobotIdentity.fromContract("Robox"));
    }
}
