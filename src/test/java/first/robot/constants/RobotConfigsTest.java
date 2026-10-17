package first.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class RobotConfigsTest {
    @Test
    void eachIdentityHasAConfig() {
        for (RobotIdentity identity : RobotIdentity.values()) {
            assertEquals(4, RobotConfigs.forIdentity(identity).drive().moduleCount(), identity.name());
        }
        assertSame(RobotConfigs.COMP_2026, RobotConfigs.forIdentity(RobotIdentity.SIM));
    }

    @Test
    void chassis2025HasItsOwnGeometryAndNoLights() {
        var config = RobotConfigs.CHASSIS_2025;
        assertTrue(config.lights().isEmpty());
        assertEquals(Units.inchesToMeters(12 * Math.sqrt(2)), config.drive().driveBaseRadiusMeters(), 1e-9);
        assertEquals(10, config.drive().drivetrain().Pigeon2Id);
        assertTrue(config.drive().maxLinearSpeedMetersPerSec()
                < RobotConfigs.COMP_2026.drive().maxLinearSpeedMetersPerSec());
    }
}
