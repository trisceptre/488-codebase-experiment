package first.lib.motor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import first.lib.LoopParticipant;
import first.lib.util.LoggedTunableNumber;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public abstract class MotorControl implements LoopParticipant, AutoCloseable {
    private static final double AT_GOAL_DEBOUNCE_SECONDS = 0.1;
    private static final double DISCONNECT_DEBOUNCE_SECONDS = 0.5;
    private static final double GAIN_RETRY_SECONDS = 1.0;

    protected final MotorConfig config;
    protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private final MotorIO io;
    private final MotorIO.Outputs outputs = new MotorIO.Outputs();
    private final Debouncer connectedDebouncer =
            new Debouncer(DISCONNECT_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);
    private final Alert disconnectedAlert;
    private final Alert followerDisconnectedAlert;
    private final Alert gainsPendingAlert;
    private final Alert seedFailedAlert;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kD;
    private double appliedKp;
    private double appliedKd;
    private double nextGainAttemptSeconds = 0.0;
    private boolean ignoreSoftLimits = false;
    private Debouncer atGoalDebouncer = newAtGoalDebouncer();
    private OptionalDouble goal = OptionalDouble.empty();
    private OptionalDouble openLoopVolts = OptionalDouble.empty();
    private boolean connected = true;
    private boolean atGoal = false;

    protected MotorControl(MotorConfig config, MotorIO io) {
        this.config = config;
        this.io = io;
        disconnectedAlert = new Alert(config.name() + " motor disconnected", Alert.AlertType.kError);
        followerDisconnectedAlert = new Alert(config.name() + " follower motor disconnected", Alert.AlertType.kWarning);
        gainsPendingAlert = new Alert(
                config.name() + " kP/kD changed but not applied; disable the robot to apply", Alert.AlertType.kInfo);
        seedFailedAlert = new Alert(config.name() + " position seed was not confirmed", Alert.AlertType.kError);
        appliedKp = config.gains().kP();
        appliedKd = config.gains().kD();
        kP = new LoggedTunableNumber(config.name() + "/kP", appliedKp, LoggedTunableNumber.Level.DEBUG);
        kD = new LoggedTunableNumber(config.name() + "/kD", appliedKd, LoggedTunableNumber.Level.DEBUG);
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
        openLoopVolts = OptionalDouble.empty();
        ignoreSoftLimits = false;
        atGoal = false;
    }

    public final void runVoltage(double volts) {
        goal = OptionalDouble.empty();
        atGoal = false;
        openLoopVolts = OptionalDouble.of(volts);
        ignoreSoftLimits = false;
    }

    /**
     * Open-loop voltage that ignores soft limits, for homing against a hard stop. It stays active until
     * {@link #stop()}, {@link #runVoltage(double)} or a new goal replaces it; seeding the position does not end it.
     */
    public final void runCalibrationVoltage(double volts) {
        runVoltage(volts);
        ignoreSoftLimits = true;
    }

    public final double velocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    public final double statorAmps() {
        return inputs.statorAmps;
    }

    public final double supplyAmps() {
        return inputs.supplyAmps;
    }

    protected final boolean seedPositionRad(double positionRad) {
        boolean seeded = io.setPosition(positionRad);
        seedFailedAlert.set(!seeded);
        if (seeded) {
            atGoal = false;
            atGoalDebouncer = newAtGoalDebouncer();
        }
        return seeded;
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
        openLoopVolts = OptionalDouble.empty();
        ignoreSoftLimits = false;
    }

    @Override
    public final void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name(), inputs);
        applyTunedGains();
        connected = connectedDebouncer.calculate(inputs.connected);
        disconnectedAlert.set(!connected);
        followerDisconnectedAlert.set(anyFalse(inputs.followerConnected));
        atGoal = atGoalDebouncer.calculate(connected && goal.isPresent() && isWithinTolerance(goal.getAsDouble()));
    }

    @Override
    public final void applyOutputs() {
        if (RobotState.isDisabled() || (goal.isEmpty() && openLoopVolts.isEmpty())) {
            outputs.setNeutral();
        } else if (openLoopVolts.isPresent()) {
            outputs.mode = MotorIO.Mode.VOLTAGE;
            outputs.value = openLoopVolts.getAsDouble();
            outputs.ignoreSoftLimits = ignoreSoftLimits;
        } else {
            outputs.ignoreSoftLimits = false;
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
        gainsPendingAlert.close();
        seedFailedAlert.close();
    }

    private void applyTunedGains() {
        double wantedKp = kP.get();
        double wantedKd = kD.get();
        boolean pending = wantedKp != appliedKp || wantedKd != appliedKd;
        double now = Timer.getTimestamp();
        if (pending && RobotState.isDisabled() && now >= nextGainAttemptSeconds) {
            if (io.setGains(wantedKp, wantedKd)) {
                appliedKp = wantedKp;
                appliedKd = wantedKd;
                pending = false;
            } else {
                nextGainAttemptSeconds = now + GAIN_RETRY_SECONDS;
            }
        }
        gainsPendingAlert.set(pending);
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
