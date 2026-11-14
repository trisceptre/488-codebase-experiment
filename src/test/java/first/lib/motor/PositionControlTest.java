package first.lib.motor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import first.lib.testing.SimTestUtil;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PositionControlTest {
    static MotorConfig armConfig() {
        return armConfig(MotorConfig.Shaping.NONE);
    }

    static MotorConfig armConfig(MotorConfig.Shaping shaping) {
        return new MotorConfig(
                "ArmTest",
                new CANId(3),
                List.of(),
                false,
                true,
                50.0,
                new MotorConfig.Gains(0.0, 0.0, 0.0, 0.5, 40.0, 2.0, MotorConfig.GravityType.ARM),
                new MotorConfig.CurrentLimits(40, 80),
                Optional.of(new MotorConfig.SoftLimits(-0.5, 1.8)),
                shaping,
                new MotorConfig.SimModel.Arm(DCMotor.getKrakenX60(1), 0.5, 0.5, -0.5, 1.8));
    }

    static final MotorConfig.Gains ELEVATOR_GAINS =
            new MotorConfig.Gains(0.0, 0.0, 0.0, 0.17, 10.0, 0.5, MotorConfig.GravityType.ELEVATOR);

    static MotorConfig elevatorConfig() {
        return elevatorConfig(ELEVATOR_GAINS, Optional.empty(), MotorConfig.Shaping.NONE);
    }

    static MotorConfig elevatorConfig(
            MotorConfig.Gains gains, Optional<MotorConfig.SoftLimits> softLimits, MotorConfig.Shaping shaping) {
        return new MotorConfig(
                "ElevatorTest",
                new CANId(4),
                List.of(),
                false,
                true,
                10.0,
                gains,
                new MotorConfig.CurrentLimits(40, 80),
                softLimits,
                shaping,
                new MotorConfig.SimModel.Elevator(DCMotor.getKrakenX60(1), 5.0, 0.02, 0.0, 1.0));
    }

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.tearDown();
    }

    private static void loops(PositionControl mechanism, int count) {
        for (int i = 0; i < count; i++) {
            SimTestUtil.step(List.of(mechanism));
        }
    }

    @Test
    void armReachesAndHoldsGoalAgainstGravity() {
        try (var arm = new PositionControl(armConfig(), new MotorIOSim(armConfig()), Degrees.of(2))) {
            arm.setGoal(Degrees.of(30));
            loops(arm, 150);
            assertTrue(arm.atGoal());
            assertEquals(30, arm.position().in(Degrees), 2);
        }
    }

    @Test
    void goalsAreClampedToSoftLimits() {
        try (var arm = new PositionControl(armConfig(), new MotorIOSim(armConfig()), Degrees.of(2))) {
            arm.setGoal(Radians.of(5.0));
            assertEquals(1.8, arm.goal().orElseThrow().in(Radians), 1e-9);
        }
    }

    @Test
    void wrongGoalTypeThrows() {
        try (var arm = new PositionControl(armConfig(), new MotorIOSim(armConfig()), Degrees.of(2));
                var elevator =
                        new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            assertThrows(IllegalStateException.class, () -> arm.setGoal(Meters.of(0.3)));
            assertThrows(IllegalStateException.class, () -> elevator.setGoal(Degrees.of(10)));
        }
    }

    @Test
    void constructorMustMatchMechanismType() {
        assertThrows(
                IllegalStateException.class,
                () -> new PositionControl(armConfig(), new MotorIOSim(armConfig()), Meters.of(0.01)));
        assertThrows(
                IllegalStateException.class,
                () -> new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Degrees.of(2)));
        new PositionControl(armConfig(), new MotorIOSim(armConfig()), Degrees.of(2)).close();
    }

    @Test
    void elevatorReachesLinearGoal() {
        try (var elevator = new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            elevator.setGoal(Meters.of(0.5));
            loops(elevator, 150);
            assertTrue(elevator.atGoal());
            assertEquals(0.5, elevator.linearPosition().in(Meters), 0.01);
        }
    }

    @Test
    void motionMagicLimitsSpeedOnTheWayToTheGoal() {
        double cruiseRadPerSec = 1.0;
        var config = armConfig(MotorConfig.Shaping.motionMagic(cruiseRadPerSec, 4.0));
        try (var arm = new PositionControl(config, new MotorIOSim(config), Degrees.of(2))) {
            arm.updateInputs();
            arm.setGoal(Radians.of(1.0));
            double fastest = 0.0;
            double last = arm.position().in(Radians);
            for (int i = 0; i < 200; i++) {
                loops(arm, 1);
                double now = arm.position().in(Radians);
                fastest = Math.max(fastest, Math.abs(now - last) / SimTestUtil.LOOP_SECONDS);
                last = now;
            }
            assertTrue(fastest <= cruiseRadPerSec * 1.25, "peak speed was " + fastest);
            assertTrue(arm.atGoal());
        }
    }

    @Test
    void runVoltageDrivesOpenLoopUntilStopped() {
        try (var elevator = new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            elevator.runVoltage(4.0);
            loops(elevator, 25);
            double raised = elevator.linearPosition().in(Meters);
            assertTrue(raised > 0.05, "height was " + raised);
            assertTrue(elevator.goal().isEmpty());
            elevator.stop();
            loops(elevator, 50);
            assertTrue(elevator.linearPosition().in(Meters) < raised);
        }
    }

    @Test
    void seedPositionOverwritesTheMeasuredPosition() {
        try (var elevator = new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            assertTrue(elevator.seedPosition(Meters.of(0.4)));
            elevator.updateInputs();
            assertEquals(0.4, elevator.linearPosition().in(Meters), 0.01);
            assertThrows(IllegalStateException.class, () -> elevator.seedPosition(Degrees.of(10)));
        }
    }

    @Test
    void seedPositionChangesTheReadingWithoutStoppingTheMechanism() {
        try (var elevator = new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            elevator.runVoltage(4.0);
            loops(elevator, 15);
            double speedBefore = elevator.velocityRadPerSec();
            assertTrue(speedBefore > 1.0, "speed was " + speedBefore);
            assertTrue(elevator.seedPosition(Meters.of(0.0)));
            loops(elevator, 1);
            assertTrue(elevator.velocityRadPerSec() > speedBefore * 0.5);
            assertTrue(elevator.linearPosition().in(Meters) < 0.05);
        }
    }

    @Test
    void seedPositionClearsAtGoal() {
        try (var elevator = new PositionControl(elevatorConfig(), new MotorIOSim(elevatorConfig()), Meters.of(0.01))) {
            elevator.setGoal(Meters.of(0.5));
            loops(elevator, 150);
            assertTrue(elevator.atGoal());
            assertTrue(elevator.seedPosition(Meters.of(0.0)));
            elevator.setGoal(Meters.of(0.5));
            assertFalse(elevator.atGoal());
        }
    }

    @Test
    void seedPositionReportsFailureWhenTheMotorIsDisconnected() {
        var io = new MotorIOSim(elevatorConfig());
        try (var elevator = new PositionControl(elevatorConfig(), io, Meters.of(0.01))) {
            io.setDisconnected(true);
            assertFalse(elevator.seedPosition(Meters.of(0.4)));
            elevator.updateInputs();
            assertEquals(0.0, elevator.linearPosition().in(Meters), 0.01);
        }
    }

    @Test
    void softLimitsStopRunVoltageButNotCalibrationVoltage() {
        double limitMeters = 0.25;
        var config = elevatorConfig(
                ELEVATOR_GAINS,
                Optional.of(new MotorConfig.SoftLimits(0.0, limitMeters / 0.02)),
                MotorConfig.Shaping.NONE);
        try (var elevator = new PositionControl(config, new MotorIOSim(config), Meters.of(0.01))) {
            elevator.runVoltage(6.0);
            double highest = 0.0;
            for (int i = 0; i < 150; i++) {
                loops(elevator, 1);
                highest = Math.max(highest, elevator.linearPosition().in(Meters));
            }
            assertTrue(highest > limitMeters - 0.02 && highest < limitMeters + 0.1, "highest was " + highest);

            elevator.runCalibrationVoltage(6.0);
            loops(elevator, 150);
            assertTrue(elevator.linearPosition().in(Meters) > limitMeters + 0.3);
        }
    }
}
