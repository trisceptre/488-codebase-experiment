package first.robot.subsystems.climber;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.lib.motor.MotorIO;
import first.lib.testing.SimTestUtil;
import first.robot.constants.Hardware2026;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ClimberTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final RecordingIO motors = new RecordingIO();
    private boolean retracted;
    private Climber climber;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        climber = new Climber(Hardware2026.CLIMBER.motors(), motors, new ClimberIO() {
            @Override
            public void updateInputs(ClimberIOInputs inputs) {
                inputs.atRetractedLimit = retracted;
            }
        });
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        climber.close();
        SimTestUtil.tearDown();
    }

    private void loops(int count) {
        for (int i = 0; i < count; i++) {
            SimTestUtil.step(List.of(climber), scheduler::run);
        }
    }

    @Test
    void retractedLimitSeedsZeroAndGoalsUseTheSeededReference() {
        loops(2);
        assertFalse(climber.isCalibrated());
        assertEquals(0, motors.seedAttempts);
        retracted = true;
        loops(2);
        assertTrue(climber.isCalibrated());
        assertEquals(0.0, climber.angleDeg(), 1e-9);
        assertEquals(1, motors.seedAttempts);

        SimTestUtil.schedule(scheduler, climber.engage());
        loops(3);
        assertEquals(MotorIO.Mode.POSITION, motors.mode);
        assertEquals(Units.degreesToRadians(85.0), motors.goal, 1e-9);
    }

    @Test
    void failedSeedDoesNotMarkCalibratedAndCanRetry() {
        retracted = true;
        motors.acceptSeed = false;
        loops(2);
        assertFalse(climber.isCalibrated());
        assertEquals(MotorIO.Mode.NEUTRAL, motors.mode);

        motors.acceptSeed = true;
        loops(2);
        assertTrue(climber.isCalibrated());
        assertEquals(0.0, climber.angleDeg(), 1e-9);
    }

    @Test
    void retractBypassesSoftLimitsOnlyUntilCalibrated() {
        SimTestUtil.schedule(scheduler, climber.retractOpenLoop());
        loops(2);
        assertEquals(MotorIO.Mode.VOLTAGE, motors.mode);
        assertTrue(motors.ignoreSoftLimits);
        retracted = true;
        loops(2);
        assertTrue(climber.isCalibrated());
        assertFalse(motors.ignoreSoftLimits);
    }

    private static class RecordingIO implements MotorIO {
        double position = 0.7;
        double goal;
        int seedAttempts;
        boolean acceptSeed = true;
        boolean ignoreSoftLimits;
        Mode mode = Mode.NEUTRAL;

        @Override
        public void updateInputs(MotorIOInputs inputs) {
            inputs.connected = true;
            inputs.positionRad = position;
        }

        @Override
        public boolean setPosition(double positionRad) {
            seedAttempts++;
            if (!acceptSeed) {
                return false;
            }
            position = positionRad;
            return true;
        }

        @Override
        public void applyOutputs(Outputs outputs) {
            mode = outputs.mode;
            goal = outputs.value;
            ignoreSoftLimits = outputs.ignoreSoftLimits;
        }
    }
}
