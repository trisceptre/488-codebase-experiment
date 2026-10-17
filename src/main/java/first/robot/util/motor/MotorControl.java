package first.robot.util.motor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import first.robot.LoopParticipant;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public abstract class MotorControl implements LoopParticipant, AutoCloseable {
    private static final double AT_GOAL_DEBOUNCE_SECONDS = 0.1;
    private static final double DISCONNECT_DEBOUNCE_SECONDS = 0.5;

    protected final MotorConfig config;
    protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private final MotorIO io;
    private final MotorIO.Outputs outputs = new MotorIO.Outputs();
    private final Debouncer connectedDebouncer =
            new Debouncer(DISCONNECT_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);
    private final Alert disconnectedAlert;
    private final Alert followerDisconnectedAlert;
    private Debouncer atGoalDebouncer = newAtGoalDebouncer();
    private OptionalDouble goal = OptionalDouble.empty();
    private boolean connected = true;
    private boolean atGoal = false;

    protected MotorControl(MotorConfig config, MotorIO io) {
        this.config = config;
        this.io = io;
        disconnectedAlert = new Alert(config.name() + " motor disconnected", Alert.AlertType.kError);
        followerDisconnectedAlert = new Alert(config.name() + " follower motor disconnected", Alert.AlertType.kWarning);
    }

    public final String name() {
        return config.name();
    }

    public final boolean isConnected() {
        return connected;
    }

    public final boolean atGoal() {
        return atGoal;
    }

    public final void stop() {
        goal = OptionalDouble.empty();
        atGoal = false;
    }

    protected final OptionalDouble goalValue() {
        return goal;
    }

    protected final void setGoalValue(double value) {
        if (goal.isEmpty() || goal.getAsDouble() != value) {
            atGoal = false;
            atGoalDebouncer = newAtGoalDebouncer();
        }
        goal = OptionalDouble.of(value);
    }

    @Override
    public final void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name(), inputs);
        connected = connectedDebouncer.calculate(inputs.connected);
        disconnectedAlert.set(!connected);
        followerDisconnectedAlert.set(anyFalse(inputs.followerConnected));
        atGoal = atGoalDebouncer.calculate(connected && goal.isPresent() && isWithinTolerance(goal.getAsDouble()));
    }

    @Override
    public final void applyOutputs() {
        if (RobotState.isDisabled() || goal.isEmpty()) {
            outputs.setNeutral();
        } else {
            fillOutputs(goal.getAsDouble(), outputs);
        }
        io.applyOutputs(outputs);
        Logger.recordOutput(name() + "/Goal", goal.orElse(0.0));
        Logger.recordOutput(name() + "/HasGoal", goal.isPresent());
        Logger.recordOutput(name() + "/AtGoal", atGoal);
        Logger.recordOutput(name() + "/Mode", outputs.mode.name());
    }

    protected abstract boolean isWithinTolerance(double goal);

    protected abstract void fillOutputs(double goal, MotorIO.Outputs outputs);

    @Override
    public void close() {
        disconnectedAlert.close();
        followerDisconnectedAlert.close();
    }

    private static Debouncer newAtGoalDebouncer() {
        return new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
    }

    private static boolean anyFalse(boolean[] values) {
        for (boolean value : values) {
            if (!value) {
                return true;
            }
        }
        return false;
    }
}
