package first.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class Hood implements Subsystem, LoopParticipant {
    // The minimum position bound of the Servo
    public static final double SERVO_MIN = 0.2;

    // The maximum position bound of the Servo
    public static final double SERVO_MAX = 0.65;

    // Time in seconds required to move from minPosition to maxPosition
    public static final double FULL_TRAVEL_SECONDS = 3.0;

    private static final double MAX_DT_SECONDS = 0.1;

    private final HoodIO io;
    private final LoggedTunableNumber minRatio = new LoggedTunableNumber("Hood/MinRatio", 0.0);
    private final LoggedTunableNumber maxRatio = new LoggedTunableNumber("Hood/MaxRatio", 1.0);
    private final LoggedTunableNumber tolerance = new LoggedTunableNumber("Hood/ToleranceRatio", 0.1);
    private final LoggedTunableNumber readinessTimeout = new LoggedTunableNumber("Hood/ReadinessTimeoutSeconds", 3.0);
    private final Debouncer atGoalDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private double goal = 0.0;
    private double estimate = 0.0;
    private double lastTimestamp = Double.NaN;
    private boolean atGoal = false;

    public Hood(HoodIO io) {
        this.io = io;
        register();
        setDefaultCommand(lowerCommand());
    }

    @Override
    public String getName() {
        return "Hood";
    }

    public Command hold(DoubleSupplier ratio) {
        return run(() -> setGoal(ratio.getAsDouble())).withName("Hood.Hold");
    }

    public Command lowerCommand() {
        return run(() -> setGoal(0.0)).ignoringDisable(true).withName("Hood.Lower");
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal)
                .withTimeout(readinessTimeout.get())
                .withName("Hood.WaitUntilAtGoal");
    }

    public void setGoal(double ratio) {
        goal = MathUtil.clamp(ratio, minRatio.get(), maxRatio.get());
    }

    public double goal() {
        return goal;
    }

    public double estimatedRatio() {
        return estimate;
    }

    public boolean atGoal() {
        return atGoal;
    }

    @Override
    public void updateInputs() {
        double now = Timer.getTimestamp();
        double dt = Double.isNaN(lastTimestamp) ? 0.0 : MathUtil.clamp(now - lastTimestamp, 0.0, MAX_DT_SECONDS);
        lastTimestamp = now;
        if (RobotState.isEnabled()) {
            double maxStep = dt / FULL_TRAVEL_SECONDS;
            estimate += MathUtil.clamp(goal - estimate, -maxStep, maxStep);
        }
        atGoal = atGoalDebouncer.calculate(Math.abs(goal - estimate) < tolerance.get());
    }

    @Override
    public void applyOutputs() {
        double position = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * goal;
        io.setServoPosition(position);
        Logger.recordOutput("Hood/GoalRatio", goal);
        Logger.recordOutput("Hood/EstimatedRatio", estimate);
        Logger.recordOutput("Hood/ServoPosition", position);
        Logger.recordOutput("Hood/AtGoal", atGoal);
    }
}
