package first.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class MotorIOSimTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static MotorIOInputsAutoLogged run(MotorIOSim io, MotorIO.Outputs outputs, int loops) {
        var inputs = new MotorIOInputsAutoLogged();
        for (int i = 0; i < loops; i++) {
            io.updateInputs(inputs);
            io.applyOutputs(outputs);
        }
        io.updateInputs(inputs);
        return inputs;
    }

    @Test
    void reportsMechanismUnitsThroughGearRatio() {
        var io = new MotorIOSim(MotorConfigTest.flywheel(2.0, List.of()));
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.VOLTAGE;
        outputs.value = 12.0;
        var inputs = run(io, outputs, 250);
        double expected = Units.rotationsPerMinuteToRadiansPerSecond(6000.0) / 2.0;
        assertEquals(expected, inputs.velocityRadPerSec, expected * 0.05);
    }

    @Test
    void velocityModeTracksSetpoint() {
        var io = new MotorIOSim(MotorConfigTest.flywheel(1.0, List.of()));
        double goalRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.VELOCITY;
        outputs.value = goalRadPerSec;
        var inputs = run(io, outputs, 150);
        assertEquals(goalRadPerSec, inputs.velocityRadPerSec, goalRadPerSec * 0.03);
    }

    @Test
    void neutralAppliesZeroVolts() {
        var io = new MotorIOSim(MotorConfigTest.flywheel(1.0, List.of()));
        var inputs = run(io, new MotorIO.Outputs(), 5);
        assertEquals(0.0, inputs.appliedVolts);
    }

    @Test
    void disconnectAndFollowersAreReported() {
        var io = new MotorIOSim(MotorConfigTest.flywheel(1.0, List.of(new MotorConfig.Follower(new CANId(2), true))));
        var inputs = run(io, new MotorIO.Outputs(), 1);
        assertTrue(inputs.connected);
        assertEquals(1, inputs.followerConnected.length);
        io.setDisconnected(true);
        inputs = run(io, new MotorIO.Outputs(), 1);
        assertFalse(inputs.connected);
        assertFalse(inputs.followerConnected[0]);
    }

    @Test
    void setGainsReplacesTheFeedbackGains() {
        var config = PositionControlTest.elevatorConfig();
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.POSITION;
        outputs.value = 25.0;

        var tuned = new MotorIOSim(config);
        assertEquals(25.0, run(tuned, outputs, 150).positionRad, 0.5);

        var detuned = new MotorIOSim(config);
        detuned.setGains(0.0, 0.0);
        assertEquals(0.0, run(detuned, outputs, 150).positionRad, 0.5);
    }

    @Test
    void rampSlewsTheAppliedVoltage() {
        var config = PositionControlTest.elevatorConfig(
                PositionControlTest.ELEVATOR_GAINS, Optional.empty(), MotorConfig.Shaping.ramp(1.0));
        var io = new MotorIOSim(config);
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.VOLTAGE;
        outputs.value = 12.0;
        assertEquals(12.0 * MotorIOSim.LOOP_PERIOD_SECONDS, run(io, outputs, 1).appliedVolts, 1e-9);
        assertEquals(6.0, run(io, outputs, 24).appliedVolts, 1e-6);
        assertEquals(12.0, run(io, outputs, 50).appliedVolts, 1e-6);
    }

    @Test
    void velocityModeAppliesGravityFeedforward() {
        var io = new MotorIOSim(PositionControlTest.elevatorConfig());
        var raise = new MotorIO.Outputs();
        raise.mode = MotorIO.Mode.VOLTAGE;
        raise.value = 4.0;
        run(io, raise, 25);

        io.setGains(0.0, 0.0);
        var hold = new MotorIO.Outputs();
        hold.mode = MotorIO.Mode.VELOCITY;
        hold.value = 0.0;
        double settled = run(io, hold, 50).positionRad;
        double later = run(io, hold, 100).positionRad;
        assertEquals(settled, later, 0.5);
    }

    @Test
    void profileFeedforwardFollowsTheProfileWithoutFeedback() {
        double kV = 12.0 / 10.0;
        var gains = new MotorConfig.Gains(0.0, kV, 0.0, 0.17, 0.0, 0.0, MotorConfig.GravityType.ELEVATOR);
        var config = PositionControlTest.elevatorConfig(
                gains, Optional.empty(), MotorConfig.Shaping.motionMagic(10.0, 40.0));
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.POSITION;
        outputs.value = 25.0;
        assertEquals(25.0, run(new MotorIOSim(config), outputs, 200).positionRad, 4.0);

        var noFeedforward = new MotorConfig.Gains(0.0, 0.0, 0.0, 0.17, 0.0, 0.0, MotorConfig.GravityType.ELEVATOR);
        var idle = PositionControlTest.elevatorConfig(
                noFeedforward, Optional.empty(), MotorConfig.Shaping.motionMagic(10.0, 40.0));
        assertEquals(0.0, run(new MotorIOSim(idle), outputs, 200).positionRad, 4.0);
    }

    @Test
    void softLimitCutsTheVoltageEvenWhileRamping() {
        var config = PositionControlTest.elevatorConfig(
                PositionControlTest.ELEVATOR_GAINS,
                Optional.of(new MotorConfig.SoftLimits(0.0, 10.0)),
                MotorConfig.Shaping.ramp(1.0));
        var io = new MotorIOSim(config);
        var outputs = new MotorIO.Outputs();
        outputs.mode = MotorIO.Mode.VOLTAGE;
        outputs.value = 8.0;
        var inputs = new MotorIOInputsAutoLogged();
        boolean reachedLimit = false;
        for (int i = 0; i < 300; i++) {
            io.updateInputs(inputs);
            io.applyOutputs(outputs);
            if (inputs.positionRad >= 10.0) {
                reachedLimit = true;
                io.updateInputs(inputs);
                assertEquals(0.0, inputs.appliedVolts, 1e-9);
                io.applyOutputs(outputs);
            }
        }
        assertTrue(reachedLimit);
    }

    @Test
    void rampOutsideThePhoenixRangeIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> MotorConfig.Shaping.ramp(1.5));
        assertThrows(IllegalArgumentException.class, () -> MotorConfig.Shaping.ramp(-0.1));
    }
}
