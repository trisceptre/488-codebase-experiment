package first.lib.motor;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;

import first.lib.testing.SimTestUtil;
import first.lib.util.LoggedTunableNumber;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

class MotorControlTest {
    private final RecordingIO io = new RecordingIO();
    private VelocityControl motor;
    private LoggedNetworkNumber kP;
    private LoggedNetworkNumber kD;

    @BeforeEach
    void setUp() throws ReflectiveOperationException {
        SimTestUtil.setUp();
        var base = MotorConfigTest.flywheel(1.0, List.of());
        var config = new MotorConfig(
                "GainTuningTest",
                base.leader(),
                base.followers(),
                base.inverted(),
                base.brakeMode(),
                base.sensorToMechanismRatio(),
                base.gains(),
                base.currentLimits(),
                base.softLimits(),
                base.shaping(),
                base.simModel());
        motor = new VelocityControl(config, io, RPM.of(50));
        kP = tunable("kP");
        kD = tunable("kD");
        kP.set(config.gains().kP());
        kD.set(config.gains().kD());
    }

    @AfterEach
    void tearDown() {
        motor.close();
        SimTestUtil.tearDown();
    }

    private void loops(int count) {
        for (int i = 0; i < count; i++) {
            // LoggedRobot normally refreshes dashboard inputs before the user loop.
            kP.periodic();
            kD.periodic();
            SimTestUtil.step(List.of(motor));
        }
    }

    private LoggedNetworkNumber tunable(String name) throws ReflectiveOperationException {
        var controlField = MotorControl.class.getDeclaredField(name);
        controlField.setAccessible(true);
        var networkField = LoggedTunableNumber.class.getDeclaredField("networkNumber");
        networkField.setAccessible(true);
        return (LoggedNetworkNumber) networkField.get(controlField.get(motor));
    }

    @Test
    void gainEditsWaitForDisableAndApplyOnce() {
        kP.set(2.0);
        kD.set(0.1);
        loops(5);
        assertEquals(0, io.attempts);

        SimTestUtil.setEnabled(false);
        loops(1);
        assertEquals(1, io.attempts);
        assertEquals(2.0, io.kP);
        assertEquals(0.1, io.kD);
        loops(60);
        assertEquals(1, io.attempts);
    }

    @Test
    void failedGainWriteRetriesAfterOneSecondAndWaitsWhileEnabled() {
        SimTestUtil.setEnabled(false);
        io.acceptGains = false;
        kP.set(3.0);
        loops(1);
        assertEquals(1, io.attempts);
        loops(40);
        assertEquals(1, io.attempts);

        SimTestUtil.setEnabled(true);
        io.acceptGains = true;
        loops(20);
        assertEquals(1, io.attempts);

        SimTestUtil.setEnabled(false);
        loops(1);
        assertEquals(2, io.attempts);
        assertEquals(3.0, io.kP);
        loops(60);
        assertEquals(2, io.attempts);
    }

    private static class RecordingIO implements MotorIO {
        int attempts;
        boolean acceptGains = true;
        double kP;
        double kD;

        @Override
        public void updateInputs(MotorIOInputs inputs) {
            inputs.connected = true;
        }

        @Override
        public boolean setGains(double kP, double kD) {
            attempts++;
            if (!acceptGains) {
                return false;
            }
            this.kP = kP;
            this.kD = kD;
            return true;
        }
    }
}
