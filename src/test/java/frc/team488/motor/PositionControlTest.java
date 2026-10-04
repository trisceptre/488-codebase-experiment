package frc.team488.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import frc.team488.testing.SimTestUtil;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

class PositionControlTest {
    static MotorConfig armConfig() {
        return new MotorConfig(
                "ArmTest",
                new CanId(3),
                List.of(),
                false,
                true,
                50.0,
                new MotorConfig.Gains(0.0, 0.0, 0.0, 0.5, 40.0, 2.0, MotorConfig.GravityType.ARM),
                new MotorConfig.CurrentLimits(40, 80),
                Optional.of(new MotorConfig.SoftLimits(-0.5, 1.8)),
                new MotorConfig.SimModel.Arm(DCMotor.getKrakenX60(1), 0.5, 0.5, -0.5, 1.8));
    }

    static MotorConfig elevatorConfig() {
        return new MotorConfig(
                "ElevatorTest",
                new CanId(4),
                List.of(),
                false,
                true,
                10.0,
                new MotorConfig.Gains(0.0, 0.0, 0.0, 0.17, 10.0, 0.5, MotorConfig.GravityType.ELEVATOR),
                new MotorConfig.CurrentLimits(40, 80),
                Optional.empty(),
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
        // A rejected construction must not leak the name's alerts.
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
}
