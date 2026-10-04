package frc.team488.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

class MotorConfigTest {
    static MotorConfig flywheel(double ratio, List<MotorConfig.Follower> followers) {
        return new MotorConfig(
                "TestFlywheel",
                new CanId(1),
                followers,
                false,
                false,
                ratio,
                new MotorConfig.Gains(0.0, 0.12, 0.0, 0.0, 0.3, 0.0, MotorConfig.GravityType.NONE),
                new MotorConfig.CurrentLimits(60, 120),
                Optional.empty(),
                new MotorConfig.SimModel.Flywheel(DCMotor.getKrakenX60(1 + followers.size()), 0.004));
    }

    @Test
    void rejectsNonPositiveRatio() {
        assertThrows(IllegalArgumentException.class, () -> flywheel(0.0, List.of()));
        assertThrows(IllegalArgumentException.class, () -> flywheel(-2.0, List.of()));
    }

    @Test
    void copiesFollowerList() {
        List<MotorConfig.Follower> followers = new ArrayList<>();
        followers.add(new MotorConfig.Follower(new CanId(2), true));
        MotorConfig config = flywheel(1.0, followers);
        followers.clear();
        assertEquals(1, config.followers().size());
    }

    @Test
    void softLimitsClampAndValidate() {
        var limits = new MotorConfig.SoftLimits(-0.5, 1.5);
        assertEquals(-0.5, limits.clamp(-3.0));
        assertEquals(1.5, limits.clamp(9.0));
        assertEquals(0.25, limits.clamp(0.25));
        assertThrows(IllegalArgumentException.class, () -> new MotorConfig.SoftLimits(1.0, 1.0));
    }
}
