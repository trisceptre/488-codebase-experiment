package first.robot.util.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import java.util.List;
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
        outputs.feedforwardVolts = 0.12 * Units.radiansToRotations(goalRadPerSec);
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
}
