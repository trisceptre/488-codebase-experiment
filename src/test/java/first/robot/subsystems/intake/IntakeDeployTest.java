package first.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.constants.Hardware2026;
import first.robot.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeDeployTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final double[] stick = {0.0};
    private IntakeDeploy intake;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        SimTestUtil.tearDown();
    }

    private void useSim() {
        intake = new IntakeDeploy(new IntakeDeployIOSim(Hardware2026.INTAKE_DEPLOY, Units.degreesToRadians(-10)));
        intake.setDefaultCommand(intake.manualControl(() -> stick[0]));
    }

    private void seconds(double s) {
        for (int i = 0; i < Math.round(s / SimTestUtil.LOOP_SECONDS); i++) {
            SimTestUtil.step(List.of(intake), scheduler::run);
        }
    }

    @Test
    void extendsAndRetractsWithThePortedGains() {
        useSim();
        seconds(0.6);
        scheduler.schedule(intake.extend());
        seconds(2.0);
        assertTrue(intake.isDeployed(), "at " + intake.positionDeg());
        assertTrue(intake.atGoal());
        scheduler.schedule(intake.retract());
        seconds(2.0);
        assertEquals(-10.0, intake.positionDeg(), 5.0);
    }

    @Test
    void manualControlWorksBothWaysAndHoldsWhereReleased() {
        useSim();
        seconds(0.6);
        stick[0] = -1.0;
        seconds(0.5);
        double lowered = intake.positionDeg();
        assertTrue(lowered < -15.0, "down stick lowers the arm, at " + lowered);
        stick[0] = 1.0;
        seconds(0.2);
        assertTrue(intake.positionDeg() > lowered, "up stick raises it");
        stick[0] = 0.0;
        seconds(0.05);
        double released = intake.positionDeg();
        assertEquals(released, intake.goalDeg(), 3.0);
        seconds(1.0);
        assertEquals(released, intake.positionDeg(), 5.0);
    }

    @Test
    void withoutTheEncoderPositionGoalsDoNothing() {
        var commanded = new String[] {"none"};
        intake = new IntakeDeploy(new IntakeDeployIO() {
            @Override
            public void setPositionGoal(double positionRad) {
                commanded[0] = "position";
            }

            @Override
            public void stop() {
                commanded[0] = "stop";
            }
        });
        scheduler.schedule(intake.extend());
        seconds(1.0);
        assertFalse(intake.isCalibrated());
        assertEquals("stop", commanded[0]);
    }

    @Test
    void adaptiveCloseHoldsBackWhenItHitsFuel() {
        var amps = new double[] {0.0};
        var position = new double[] {Units.degreesToRadians(-140)};
        intake = new IntakeDeploy(new IntakeDeployIO() {
            @Override
            public void updateInputs(IntakeDeployIOInputs inputs) {
                inputs.encoderConnected = true;
                inputs.motorConnected = true;
                inputs.positionRad = position[0];
                inputs.statorAmps = amps[0];
            }
        });
        seconds(0.6);
        SimTestUtil.schedule(scheduler, intake.adaptiveCloseWhileFiring());
        seconds(0.2);
        double advancing = intake.goalDeg();
        assertTrue(advancing > -140.0, "advances upward, goal " + advancing);

        position[0] = Units.degreesToRadians(-130);
        amps[0] = 40.0;
        seconds(0.4);
        assertEquals(-130.0, intake.goalDeg(), 15.0 + 1e-6);
        amps[0] = 0.0;
        seconds(3.0);
        assertEquals(-90.0, intake.goalDeg(), 15.0 + 1e-6);
    }
}
