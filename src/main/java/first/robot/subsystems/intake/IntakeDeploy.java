package first.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class IntakeDeploy implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double NOMINAL_VOLTS = 12.0;
    private static final double MANUAL_DEADBAND = 0.15;
    private static final double MAX_DT_SECONDS = 0.1;

    private final IntakeDeployIO io;
    private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

    private final LoggedTunableNumber extendedDeg = new LoggedTunableNumber("IntakeDeploy/ExtendedDeg", -145.0);
    private final LoggedTunableNumber retractedDeg = new LoggedTunableNumber("IntakeDeploy/RetractedDeg", -10.0);
    private final LoggedTunableNumber manualPower = new LoggedTunableNumber("IntakeDeploy/ManualPower", 0.2);
    private final LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("IntakeDeploy/ToleranceDeg", 5.0);
    private final LoggedTunableNumber readinessTimeout =
            new LoggedTunableNumber("IntakeDeploy/ReadinessTimeoutSeconds", 3.0);

    private final LoggedTunableNumber collectBaseDeg = new LoggedTunableNumber("IntakeDeploy/Collect/BaseDeg", -134.0);
    private final LoggedTunableNumber collectAmplitudeDeg =
            new LoggedTunableNumber("IntakeDeploy/Collect/AmplitudeDeg", 2.0);
    private final LoggedTunableNumber collectPeriod =
            new LoggedTunableNumber("IntakeDeploy/Collect/PeriodSeconds", 1.0);

    // Advance rate and limits
    private final LoggedTunableNumber adaptiveRateDeg =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/AdvanceDegPerSec", 90.0);
    private final LoggedTunableNumber adaptiveLimitDeg =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/RetractLimitDeg", -90.0);

    // Current spike detection
    private final LoggedTunableNumber adaptiveStallAmps =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/StallAmps", 15.0);

    // Oscillation parameters (used while stalled)
    private final LoggedTunableNumber adaptiveAmplitudeDeg =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/AmplitudeDeg", 15.0);
    private final LoggedTunableNumber adaptivePeriod =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/PeriodSeconds", 0.75);

    // How long current must stay above threshold to confirm a stall
    private final LoggedTunableNumber adaptiveStallSeconds =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/StallSeconds", 0.25);

    // Dwell time after detecting a stall before resuming advance
    private final LoggedTunableNumber adaptiveDwellSeconds =
            new LoggedTunableNumber("IntakeDeploy/AdaptiveClose/DwellSeconds", 0.75);

    private final LoggedTunableNumber closeOscRateDeg =
            new LoggedTunableNumber("IntakeDeploy/CloseOscillating/DegPerSec", 25.0);
    private final LoggedTunableNumber closeOscLimitDeg =
            new LoggedTunableNumber("IntakeDeploy/CloseOscillating/RetractLimitDeg", -90.0);
    private final LoggedTunableNumber closeOscAmplitudeDeg =
            new LoggedTunableNumber("IntakeDeploy/CloseOscillating/AmplitudeDeg", 15.0);
    private final LoggedTunableNumber closeOscPeriod =
            new LoggedTunableNumber("IntakeDeploy/CloseOscillating/PeriodSeconds", 1.0);

    private final LoggedTunableNumber slowCloseRateDeg =
            new LoggedTunableNumber("IntakeDeploy/SlowClose/DegPerSec", 70.0);
    private final LoggedTunableNumber slowCloseLimitDeg =
            new LoggedTunableNumber("IntakeDeploy/SlowClose/RetractLimitDeg", -30.0);

    private final Debouncer encoderDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer atGoalDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    private double goalDeg;
    private OptionalDouble manualVolts = OptionalDouble.empty();
    private boolean calibrated = false;
    private boolean atGoal = false;

    public IntakeDeploy(IntakeDeployIO io) {
        this.io = io;
        goalDeg = retractedDeg.get();
        register();
        setDefaultCommand(holdCommand());
    }

    @Override
    public String getName() {
        return "IntakeDeploy";
    }

    public Command manualControl(DoubleSupplier stickUp) {
        var wasManual = new boolean[] {false};
        return run(() -> {
                    double stick = MathUtil.applyDeadband(stickUp.getAsDouble(), MANUAL_DEADBAND);
                    if (stick != 0.0) {
                        manualVolts = OptionalDouble.of(stick * manualPower.get() * NOMINAL_VOLTS);
                        wasManual[0] = true;
                        return;
                    }
                    manualVolts = OptionalDouble.empty();
                    // only do this if we're not in autonomous, otherwise it can replace a goal set by auto right at
                    // the start
                    if (wasManual[0] && !DriverStation.isAutonomous()) {
                        goalDeg = positionDeg();
                    }
                    wasManual[0] = false;
                })
                .finallyDo(() -> manualVolts = OptionalDouble.empty())
                .withName("IntakeDeploy.ManualControl");
    }

    public Command holdCommand() {
        return run(() -> manualVolts = OptionalDouble.empty()).withName("IntakeDeploy.Hold");
    }

    public Command extend() {
        return runOnce(() -> goalDeg = extendedDeg.get()).withName("IntakeDeploy.Extend");
    }

    public Command retract() {
        return runOnce(() -> goalDeg = retractedDeg.get()).withName("IntakeDeploy.Retract");
    }

    public Command goTo(double degrees) {
        return runOnce(() -> goalDeg = degrees).withName("IntakeDeploy.GoTo");
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal)
                .withTimeout(readinessTimeout.get())
                .withName("IntakeDeploy.WaitUntilAtGoal");
    }

    public Command oscillateForCollection() {
        return new OscillateForCollection().withName("IntakeDeploy.OscillateForCollection");
    }

    public Command adaptiveCloseWhileFiring() {
        return new AdaptiveClose().withName("IntakeDeploy.AdaptiveCloseWhileFiring");
    }

    public Command closeWhileOscillating() {
        return new CloseWhileOscillating().withName("IntakeDeploy.CloseWhileOscillating");
    }

    public Command slowClose() {
        return new SlowClose().withName("IntakeDeploy.SlowClose");
    }

    public double positionDeg() {
        return Units.radiansToDegrees(inputs.positionRad);
    }

    public double goalDeg() {
        return goalDeg;
    }

    public double statorAmps() {
        return inputs.statorAmps;
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public boolean atGoal() {
        return atGoal;
    }

    public boolean isDeployed() {
        return Math.abs(positionDeg() - extendedDeg.get()) <= toleranceDeg.get();
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeDeploy", inputs);
        calibrated = encoderDebouncer.calculate(inputs.encoderConnected);
        atGoal = atGoalDebouncer.calculate(calibrated && Math.abs(goalDeg - positionDeg()) <= toleranceDeg.get());
    }

    @Override
    public void applyOutputs() {
        String mode;
        if (RobotState.isDisabled()) {
            io.stop();
            mode = "Disabled";
        } else if (manualVolts.isPresent()) {
            io.setVoltage(manualVolts.getAsDouble());
            mode = "Manual";
        } else if (calibrated) {
            io.setPositionGoal(Units.degreesToRadians(goalDeg));
            mode = "Position";
        } else {
            io.stop();
            mode = "Uncalibrated";
        }
        Logger.recordOutput("IntakeDeploy/Mode", mode);
        Logger.recordOutput("IntakeDeploy/GoalDeg", goalDeg);
        Logger.recordOutput("IntakeDeploy/PositionDeg", positionDeg());
        Logger.recordOutput("IntakeDeploy/AtGoal", atGoal);
    }

    @Override
    public void close() {
        if (io instanceof AutoCloseable closeable) {
            try {
                closeable.close();
            } catch (Exception e) {
                throw new IllegalStateException(e);
            }
        }
    }

    private static double oscillation(double amplitudeDeg, double periodSeconds, double elapsedSeconds) {
        return amplitudeDeg * Math.sin(2 * Math.PI * elapsedSeconds / periodSeconds);
    }

    private static double dt(double last, double now) {
        return MathUtil.clamp(now - last, 0.0, MAX_DT_SECONDS);
    }

    private final class OscillateForCollection extends Command {
        private double startTime;

        OscillateForCollection() {
            addRequirements(IntakeDeploy.this);
        }

        @Override
        public void initialize() {
            // Record when the command started so we can measure elapsed time in execute().
            startTime = Timer.getTimestamp();
        }

        @Override
        public void execute() {
            // The intake oscillates around this command's own base position.
            goalDeg = collectBaseDeg.get()
                    + oscillation(collectAmplitudeDeg.get(), collectPeriod.get(), Timer.getTimestamp() - startTime);
        }

        @Override
        public void end(boolean interrupted) {
            // Stop the oscillation and hold the intake at the extended position.
            goalDeg = extendedDeg.get();
        }
    }

    /**
     * Adaptively raises the intake to push balls toward the shooter. Advances upward until a current
     * spike is detected (indicating contact with balls), then pauses and oscillates to loosen them
     * before resuming. Repeats until reaching the retract limit.
     */
    private final class AdaptiveClose extends Command {
        private enum State {
            ADVANCING,
            STALLED,
            AT_LIMIT_OSCILLATING
        }

        private final Debouncer stallDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
        private State state;
        private double baseDeg;
        private double stateStart;
        private double lastTime;

        AdaptiveClose() {
            addRequirements(IntakeDeploy.this);
        }

        @Override
        public void initialize() {
            stallDebouncer.setDebounceTime(adaptiveStallSeconds.get());
            stallDebouncer.calculate(false);
            baseDeg = positionDeg();
            lastTime = Timer.getTimestamp();
            changeState(State.ADVANCING);
        }

        @Override
        public void execute() {
            double now = Timer.getTimestamp();
            boolean currentIsStableHigh = stallDebouncer.calculate(statorAmps() > adaptiveStallAmps.get());
            switch (state) {
                case ADVANCING -> {
                    // Move base position toward retract limit
                    baseDeg += adaptiveRateDeg.get() * dt(lastTime, now);
                    if (baseDeg >= adaptiveLimitDeg.get()) {
                        baseDeg = adaptiveLimitDeg.get();
                        changeState(State.AT_LIMIT_OSCILLATING);
                    }

                    // Transition to stalled only after current has been high for the stable window
                    if (currentIsStableHigh && state == State.ADVANCING) {
                        baseDeg = positionDeg(); // lock in position at stall
                        changeState(State.STALLED);
                    }

                    goalDeg = baseDeg;
                }
                case STALLED -> {
                    // Oscillate around the stall position to loosen balls
                    goalDeg = baseDeg + oscillationOffset(now);

                    // After dwell time, resume advancing
                    if (now - stateStart >= adaptiveDwellSeconds.get()) {
                        changeState(State.ADVANCING);
                    }
                }
                case AT_LIMIT_OSCILLATING -> {
                    // Reached the retract limit — just oscillate indefinitely
                    goalDeg = adaptiveLimitDeg.get() + oscillationOffset(now);
                }
            }
            lastTime = now;
            Logger.recordOutput("IntakeDeploy/AdaptiveClose/State", state.name());
            Logger.recordOutput("IntakeDeploy/AdaptiveClose/BaseDeg", baseDeg);
        }

        @Override
        public void end(boolean interrupted) {
            // leave the target in the current base position
            goalDeg = baseDeg;
        }

        private double oscillationOffset(double now) {
            return oscillation(adaptiveAmplitudeDeg.get(), adaptivePeriod.get(), now - stateStart);
        }

        private void changeState(State next) {
            state = next;
            stateStart = Timer.getTimestamp();
        }
    }

    private final class CloseWhileOscillating extends Command {
        private double startTime;
        private double slowCloseTargetDeg;
        private double lastTime;

        CloseWhileOscillating() {
            addRequirements(IntakeDeploy.this);
        }

        @Override
        public void initialize() {
            startTime = Timer.getTimestamp();
            lastTime = startTime;
            slowCloseTargetDeg = positionDeg();
        }

        @Override
        public void execute() {
            double now = Timer.getTimestamp();
            slowCloseTargetDeg =
                    Math.min(slowCloseTargetDeg + closeOscRateDeg.get() * dt(lastTime, now), closeOscLimitDeg.get());
            lastTime = now;
            goalDeg =
                    slowCloseTargetDeg + oscillation(closeOscAmplitudeDeg.get(), closeOscPeriod.get(), now - startTime);
        }
    }

    private final class SlowClose extends Command {
        private double currentTargetDeg;
        private double lastTime;

        SlowClose() {
            addRequirements(IntakeDeploy.this);
        }

        @Override
        public void initialize() {
            // Start the running target from the mechanism's current position so the
            // first execute() call doesn't jump the setpoint.
            currentTargetDeg = positionDeg();
            lastTime = Timer.getTimestamp();
        }

        @Override
        public void execute() {
            // Increment our internal target each loop rather than reading the encoder,
            // so the setpoint moves smoothly regardless of where the mechanism actually is.
            double now = Timer.getTimestamp();
            currentTargetDeg += slowCloseRateDeg.get() * dt(lastTime, now);
            lastTime = now;
            goalDeg = Math.min(currentTargetDeg, slowCloseLimitDeg.get());
        }
    }
}
