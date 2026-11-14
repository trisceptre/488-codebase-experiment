package first.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class MotorIOTalonFXConfigTest {
    private static MotorConfig config(
            MotorConfig.Gains gains, MotorConfig.CurrentLimits limits, MotorConfig.Shaping shaping) {
        return new MotorConfig(
                "ConfigTest",
                new CANId(9),
                List.of(),
                false,
                true,
                10.0,
                gains,
                limits,
                Optional.of(new MotorConfig.SoftLimits(-Math.PI, Math.PI)),
                shaping,
                new MotorConfig.SimModel.Flywheel(DCMotor.getKrakenX60(1), 0.004));
    }

    private static final MotorConfig.Gains ARM_GAINS =
            new MotorConfig.Gains(0.1, 0.2, 0.3, 0.4, 5.0, 0.6, MotorConfig.GravityType.ARM);

    @Test
    void feedforwardAndFeedbackGainsGoToSlot0() {
        var talon = MotorIOTalonFX.toTalonConfig(
                config(ARM_GAINS, new MotorConfig.CurrentLimits(40, 80), MotorConfig.Shaping.NONE));
        assertEquals(0.1, talon.Slot0.kS);
        assertEquals(0.2, talon.Slot0.kV);
        assertEquals(0.3, talon.Slot0.kA);
        assertEquals(0.4, talon.Slot0.kG);
        assertEquals(5.0, talon.Slot0.kP);
        assertEquals(0.6, talon.Slot0.kD);
        assertEquals(GravityTypeValue.Arm_Cosine, talon.Slot0.GravityType);
        assertEquals(-0.5, talon.SoftwareLimitSwitch.ReverseSoftLimitThreshold, 1e-9);
        assertEquals(0.5, talon.SoftwareLimitSwitch.ForwardSoftLimitThreshold, 1e-9);
    }

    @Test
    void gravityGainIsDroppedWhenThereIsNoGravityType() {
        var gains = new MotorConfig.Gains(0.0, 0.0, 0.0, 0.4, 1.0, 0.0, MotorConfig.GravityType.NONE);
        var talon = MotorIOTalonFX.toTalonConfig(
                config(gains, new MotorConfig.CurrentLimits(40, 80), MotorConfig.Shaping.NONE));
        assertEquals(0.0, talon.Slot0.kG);
    }

    @Test
    void lowerSupplyLimitKeepsThePhoenixDefaultUnlessGiven() {
        var defaults = new TalonFXConfiguration().CurrentLimits;
        var untouched = MotorIOTalonFX.toTalonConfig(
                        config(ARM_GAINS, new MotorConfig.CurrentLimits(40, 80), MotorConfig.Shaping.NONE))
                .CurrentLimits;
        assertEquals(defaults.SupplyCurrentLowerLimit, untouched.SupplyCurrentLowerLimit);
        assertEquals(defaults.SupplyCurrentLowerTime, untouched.SupplyCurrentLowerTime);
        assertTrue(untouched.SupplyCurrentLimitEnable);

        var limits = new MotorConfig.CurrentLimits(40, 80, Optional.of(new MotorConfig.SupplyLowerLimit(25, 0.5)));
        var given = MotorIOTalonFX.toTalonConfig(config(ARM_GAINS, limits, MotorConfig.Shaping.NONE)).CurrentLimits;
        assertEquals(25.0, given.SupplyCurrentLowerLimit);
        assertEquals(0.5, given.SupplyCurrentLowerTime);

        var from2026Constructor = MotorIOTalonFX.toTalonConfig(
                        config(ARM_GAINS, new MotorConfig.CurrentLimits(40, 80, 25, 0.5), MotorConfig.Shaping.NONE))
                .CurrentLimits;
        assertEquals(given.SupplyCurrentLowerLimit, from2026Constructor.SupplyCurrentLowerLimit);
        assertEquals(given.SupplyCurrentLowerTime, from2026Constructor.SupplyCurrentLowerTime);
    }

    @Test
    void shapingSetsRampsAndMotionMagicInRotations() {
        var ramped = MotorIOTalonFX.toTalonConfig(
                config(ARM_GAINS, new MotorConfig.CurrentLimits(40, 80), MotorConfig.Shaping.ramp(0.3)));
        assertEquals(0.3, ramped.OpenLoopRamps.VoltageOpenLoopRampPeriod);
        assertEquals(0.3, ramped.ClosedLoopRamps.VoltageClosedLoopRampPeriod);

        var profiled = MotorIOTalonFX.toTalonConfig(config(
                ARM_GAINS,
                new MotorConfig.CurrentLimits(40, 80),
                MotorConfig.Shaping.motionMagic(2.0 * Math.PI, 4.0 * Math.PI)));
        assertEquals(1.0, profiled.MotionMagic.MotionMagicCruiseVelocity, 1e-9);
        assertEquals(2.0, profiled.MotionMagic.MotionMagicAcceleration, 1e-9);
    }
}
