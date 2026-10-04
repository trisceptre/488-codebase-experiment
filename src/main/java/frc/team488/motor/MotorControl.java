package frc.team488.motor;

import frc.team488.LoopParticipant;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.util.Alert;

/**
 * Shared behavior for motor-driven mechanisms: input logging, disconnect alerts, a debounced
 * at-goal flag, and NEUTRAL output whenever the robot is disabled or no goal is set. It doesn't
 * depend on the command framework, so commands or a state machine can drive it.
 *
 * <p>Alert IDs derive from the config name, so two live mechanisms can't share a name. {@link
 * #close()} releases them (tests construct many mechanisms).
 */
public abstract class MotorControl implements LoopParticipant, AutoCloseable {
    private static final double AT_GOAL_DEBOUNCE_SECONDS = 0.1;
    private static final double DISCONNECT_DEBOUNCE_SECONDS = 0.5;

    protected final MotorConfig config;
    protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private final MotorIO io;
    private final MotorIO.Outputs outputs = new MotorIO.Outputs();
    private final Debouncer connectedDebouncer =
            new Debouncer(DISCONNECT_DEBOUNCE_SECONDS, Debouncer.DebounceType.FALLING);
    private final Alert disconnectedAlert;
    private final Alert followerDisconnectedAlert;
    private Debouncer atGoalDebouncer = newAtGoalDebouncer();
    private OptionalDouble goal = OptionalDouble.empty();
    private boolean connected = true;
    private boolean atGoal = false;

    protected MotorControl(MotorConfig config, MotorIO io) {
        this.config = config;
        this.io = io;
        disconnectedAlert =
                new Alert(config.name() + "/Disconnected", config.name() + " motor disconnected", Alert.Level.HIGH);
        followerDisconnectedAlert = new Alert(
                config.name() + "/FollowerDisconnected",
                config.name() + " follower motor disconnected",
                Alert.Level.MEDIUM);
    }

    public final String name() {
        return config.name();
    }

    /** False once the leader has reported disconnected for 0.5 s. */
    public final boolean isConnected() {
        return connected;
    }

    /** True once the mechanism has been within tolerance of its goal for 0.1 s while connected. */
    public final boolean atGoal() {
        return atGoal;
    }

    /** Clears the goal; outputs go NEUTRAL. */
    public final void stop() {
        goal = OptionalDouble.empty();
        atGoal = false;
    }

    protected final OptionalDouble goalValue() {
        return goal;
    }

    /** Sets the goal in mechanism units (radians or radians/s). A new value restarts the at-goal debounce. */
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

    /** Whether the latest inputs are within tolerance of {@code goal} (mechanism units). */
    protected abstract boolean isWithinTolerance(double goal);

    /** Fills a non-neutral output request for {@code goal} (mechanism units). */
    protected abstract void fillOutputs(double goal, MotorIO.Outputs outputs);

    /** Releases this mechanism's alerts. */
    @Override
    public void close() {
        disconnectedAlert.close();
        followerDisconnectedAlert.close();
    }

    private static Debouncer newAtGoalDebouncer() {
        return new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);
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
