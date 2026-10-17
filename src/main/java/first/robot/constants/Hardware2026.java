package first.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import first.robot.subsystems.intake.IntakeDeployConfig;
import first.robot.util.motor.CANId;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorConfig.CurrentLimits;
import first.robot.util.motor.MotorConfig.Gains;
import first.robot.util.motor.MotorConfig.GravityType;
import first.robot.util.motor.MotorConfig.Shaping;
import first.robot.util.motor.MotorConfig.SimModel;
import java.util.List;
import java.util.Optional;

public final class Hardware2026 {
    private Hardware2026() {}

    private static final String CANIVORE = "*";
    private static final double VOLTS_PER_DUTY = 12.0;
    private static final double DEFAULT_SUPPLY_AMPS = 70.0;
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    private static CANId canivore(int id) {
        return new CANId(id, CANIVORE);
    }

    private static Gains dutyCycleVelocityGains(double kP, double kS, double kV) {
        return new Gains(
                kS * VOLTS_PER_DUTY, kV * VOLTS_PER_DUTY, 0.0, 0.0, kP * VOLTS_PER_DUTY, 0.0, GravityType.NONE);
    }

    private static MotorConfig roller(
            String name, int id, boolean inverted, Gains gains, CurrentLimits limits, Shaping shaping, double moiKgM2) {
        return new MotorConfig(
                name,
                canivore(id),
                List.of(),
                inverted,
                false,
                1.0,
                gains,
                limits,
                Optional.empty(),
                shaping,
                new SimModel.Flywheel(KRAKEN, moiKgM2));
    }

    private static final CurrentLimits SHOOTER_LIMITS = new CurrentLimits(60, 50, 40, 1.0);

    public static final List<MotorConfig> SHOOTER = List.of(
            roller(
                    "Shooter/Left",
                    22,
                    false,
                    dutyCycleVelocityGains(0.05, 0.02, 0.01),
                    SHOOTER_LIMITS,
                    Shaping.ramp(0.2),
                    0.0025),
            roller(
                    "Shooter/Middle",
                    23,
                    false,
                    dutyCycleVelocityGains(0.05, 0.02, 0.01),
                    SHOOTER_LIMITS,
                    Shaping.ramp(0.2),
                    0.0025),
            roller(
                    "Shooter/Right",
                    24,
                    true,
                    dutyCycleVelocityGains(0.05, 0.02, 0.011),
                    SHOOTER_LIMITS,
                    Shaping.ramp(0.2),
                    0.0025));

    public static final MotorConfig FEEDER = roller(
            "Feeder/Motor",
            37,
            true,
            dutyCycleVelocityGains(0.0, 0.0, 0.1),
            new CurrentLimits(DEFAULT_SUPPLY_AMPS, 35),
            Shaping.ramp(0.1),
            0.0005);

    public static final MotorConfig HOPPER = roller(
            "Hopper/Motor",
            33,
            false,
            dutyCycleVelocityGains(0.001, 0.05, 0.008),
            new CurrentLimits(DEFAULT_SUPPLY_AMPS, 40),
            Shaping.ramp(0.1),
            0.001);

    public static final MotorConfig COLLECTOR = roller(
            "Collector/Motor",
            32,
            true,
            dutyCycleVelocityGains(0.05, 0.02, 0.01),
            new CurrentLimits(DEFAULT_SUPPLY_AMPS, 70),
            Shaping.NONE,
            0.001);

    public static final IntakeDeployConfig INTAKE_DEPLOY = new IntakeDeployConfig(
            canivore(34), canivore(58), false, 40.0, 0.4 * VOLTS_PER_DUTY, 200.0, 200.0, 0.1, 50, 30, 15, 1.0);

    private static final double CLIMBER_RATIO = 120.0;

    public static final Superstructure.Climber CLIMBER = new Superstructure.Climber(
            new MotorConfig(
                    "Climber/Motors",
                    canivore(25),
                    List.of(new MotorConfig.Follower(canivore(26), true)),
                    false,
                    false,
                    CLIMBER_RATIO,
                    new Gains(0.0, 0.0, 0.0, 0.0, 0.6 * CLIMBER_RATIO, 0.0, GravityType.NONE),
                    new CurrentLimits(60, 80, 40, 1.0),
                    Optional.empty(),
                    Shaping.motionMagic(
                            Units.rotationsToRadians(100.0 / CLIMBER_RATIO),
                            Units.rotationsToRadians(300.0 / CLIMBER_RATIO)),
                    new SimModel.Arm(DCMotor.getKrakenX60(2), 0.5, 0.5, -0.1, Math.PI + 0.3)),
            0);

    public static final Superstructure SUPERSTRUCTURE =
            new Superstructure(SHOOTER, 7, 9, FEEDER, HOPPER, COLLECTOR, INTAKE_DEPLOY, Optional.empty());
}
