package frc.team488.motor;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import frc.team488.util.TunableNumber;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A position-controlled mechanism: an arm or pivot (rotary, goals as {@link Angle}) or an elevator
 * (linear, goals as {@link Distance}, which requires a {@code SimModel.Elevator} for the drum
 * radius). Goals are clamped to the config's soft limits. Feedforward is kG by gravity type.
 */
public class PositionControl extends MotorControl {
    private final TunableNumber toleranceRad;
    private final boolean linear;
    /** Drum radius for linear mechanisms; unused (0) for rotary ones. */
    private final double drumRadiusMeters;

    /** A rotary mechanism. */
    public PositionControl(MotorConfig config, MotorIO io, Angle tolerance) {
        // Checked before super() so a rejected config never allocates the name's alerts.
        if (drumRadius(config).isPresent()) {
            throw new IllegalStateException(config.name() + " is linear; construct it with a Distance tolerance");
        }
        super(config, io);
        linear = false;
        drumRadiusMeters = 0.0;
        toleranceRad = new TunableNumber(name() + "/ToleranceRad", tolerance.in(Radians));
    }

    /** A linear mechanism; its config must use {@code SimModel.Elevator}. */
    public PositionControl(MotorConfig config, MotorIO io, Distance tolerance) {
        if (drumRadius(config).isEmpty()) {
            throw new IllegalStateException(config.name() + " is rotary; construct it with an Angle tolerance");
        }
        super(config, io);
        linear = true;
        drumRadiusMeters = drumRadius(config).getAsDouble();
        toleranceRad = new TunableNumber(name() + "/ToleranceRad", tolerance.in(Meters) / drumRadiusMeters);
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

    /** The goal as a mechanism (or drum) angle. */
    public Optional<Angle> goal() {
        OptionalDouble goal = goalValue();
        return goal.isPresent() ? Optional.of(Radians.of(goal.getAsDouble())) : Optional.empty();
    }

    /** The mechanism (or drum) angle. */
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

    private static OptionalDouble drumRadius(MotorConfig config) {
        return config.simModel() instanceof MotorConfig.SimModel.Elevator elevator
                ? OptionalDouble.of(elevator.drumRadiusMeters())
                : OptionalDouble.empty();
    }
}
