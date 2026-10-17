package first.robot.util.motor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import first.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class PositionControl extends MotorControl {
    private final LoggedTunableNumber toleranceRad;
    private final boolean linear;
    private final double drumRadiusMeters;

    public PositionControl(MotorConfig config, MotorIO io, Angle tolerance) {
        super(requireRotary(config), io);
        linear = false;
        drumRadiusMeters = 0.0;
        toleranceRad = new LoggedTunableNumber(name() + "/ToleranceRad", tolerance.in(Radians));
    }

    public PositionControl(MotorConfig config, MotorIO io, Distance tolerance) {
        super(requireLinear(config), io);
        linear = true;
        drumRadiusMeters = drumRadius(config).getAsDouble();
        toleranceRad = new LoggedTunableNumber(name() + "/ToleranceRad", tolerance.in(Meters) / drumRadiusMeters);
    }

    public boolean isLinear() {
        return linear;
    }

    public void setGoal(Angle angle) {
        if (isLinear()) {
            throw new IllegalStateException(name() + " is linear; use setGoal(Distance)");
        }
        setClampedGoal(angle.in(Radians));
    }

    public void setGoal(Distance height) {
        if (!isLinear()) {
            throw new IllegalStateException(name() + " is rotary; use setGoal(Angle)");
        }
        setClampedGoal(height.in(Meters) / drumRadiusMeters);
    }

    public Optional<Angle> goal() {
        OptionalDouble goal = goalValue();
        return goal.isPresent() ? Optional.of(Radians.of(goal.getAsDouble())) : Optional.empty();
    }

    public Angle position() {
        return Radians.of(inputs.positionRad);
    }

    public Distance linearPosition() {
        if (!isLinear()) {
            throw new IllegalStateException(name() + " is rotary; use position()");
        }
        return Meters.of(inputs.positionRad * drumRadiusMeters);
    }

    @Override
    protected boolean isWithinTolerance(double goal) {
        return Math.abs(inputs.positionRad - goal) <= toleranceRad.get();
    }

    @Override
    protected void fillOutputs(double goal, MotorIO.Outputs outputs) {
        MotorConfig.Gains gains = config.gains();
        outputs.mode = MotorIO.Mode.POSITION;
        outputs.value = goal;
        outputs.feedforwardVolts = switch (gains.gravityType()) {
            case NONE -> 0.0;
            case ELEVATOR -> gains.kG();
            case ARM -> gains.kG() * Math.cos(inputs.positionRad);
        };
    }

    private void setClampedGoal(double rad) {
        double clamped = config.softLimits().map(limits -> limits.clamp(rad)).orElse(rad);
        Logger.recordOutput(name() + "/GoalClamped", clamped != rad);
        setGoalValue(clamped);
    }

    private static MotorConfig requireRotary(MotorConfig config) {
        if (drumRadius(config).isPresent()) {
            throw new IllegalStateException(config.name() + " is linear; construct it with a Distance tolerance");
        }
        return config;
    }

    private static MotorConfig requireLinear(MotorConfig config) {
        if (drumRadius(config).isEmpty()) {
            throw new IllegalStateException(config.name() + " is rotary; construct it with an Angle tolerance");
        }
        return config;
    }

    private static OptionalDouble drumRadius(MotorConfig config) {
        return config.simModel() instanceof MotorConfig.SimModel.Elevator elevator
                ? OptionalDouble.of(elevator.drumRadiusMeters())
                : OptionalDouble.empty();
    }
}
