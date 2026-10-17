package first.robot.util.motor;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import first.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.OptionalDouble;

public class VelocityControl extends MotorControl {
    private final LoggedTunableNumber toleranceRpm;

    public VelocityControl(MotorConfig config, MotorIO io, AngularVelocity tolerance) {
        super(config, io);
        toleranceRpm = new LoggedTunableNumber(config.name() + "/ToleranceRPM", tolerance.in(RPM));
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
