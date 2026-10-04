package frc.team488.motor;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import frc.team488.util.TunableNumber;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;

/** A velocity-controlled mechanism such as a flywheel or roller. Feedforward: kS + kV. */
public class VelocityControl extends MotorControl {
    private final TunableNumber toleranceRpm;

    public VelocityControl(MotorConfig config, MotorIO io, AngularVelocity tolerance) {
        super(config, io);
        toleranceRpm = new TunableNumber(config.name() + "/ToleranceRPM", tolerance.in(RPM));
    }

    public void setGoal(AngularVelocity velocity) {
        setGoalValue(velocity.in(RadiansPerSecond));
    }

    public Optional<AngularVelocity> goal() {
        OptionalDouble goal = goalValue();
        return goal.isPresent() ? Optional.of(RadiansPerSecond.of(goal.getAsDouble())) : Optional.empty();
    }

    public AngularVelocity velocity() {
        return RadiansPerSecond.of(inputs.velocityRadPerSec);
    }

    @Override
    protected boolean isWithinTolerance(double goal) {
        return Math.abs(inputs.velocityRadPerSec - goal)
                <= RPM.of(toleranceRpm.get()).in(RadiansPerSecond);
    }

    @Override
    protected void fillOutputs(double goal, MotorIO.Outputs outputs) {
        double goalRotationsPerSec = Units.radiansToRotations(goal);
        MotorConfig.Gains gains = config.gains();
        outputs.mode = MotorIO.Mode.VELOCITY;
        outputs.value = goal;
        outputs.feedforwardVolts = gains.kS() * Math.signum(goalRotationsPerSec) + gains.kV() * goalRotationsPerSec;
    }
}
