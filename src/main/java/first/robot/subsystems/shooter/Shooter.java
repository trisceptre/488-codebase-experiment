package first.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.VelocityControl;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class Shooter implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double TRIM_STEP_RPM = 15.0;
    private static final double IDLE_RPM = 300.0;

    private final List<VelocityControl> motors = new ArrayList<>();
    private final LoggedTunableNumber defaultRpm = new LoggedTunableNumber("Shooter/DefaultRPM", 3000.0);
    private final LoggedTunableNumber savedTrimRpm = new LoggedTunableNumber("Shooter/TrimRPM", 0.0);
    private final LoggedTunableNumber toleranceRpm = new LoggedTunableNumber("Shooter/ToleranceRPM", 300.0);
    private final Debouncer atGoalDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    private double goalRpm = 0.0;
    private double trimRpm = 0.0;
    private boolean lowPowerMode = false;
    private boolean atGoal = false;
    private List<VelocityControl> active = List.of();

    public Shooter(List<MotorConfig> configs, List<MotorIO> ios) {
        if (configs.size() != 3 || ios.size() != 3) {
            throw new IllegalArgumentException("shooter needs left, middle and right motors");
        }
        for (int i = 0; i < configs.size(); i++) {
            motors.add(new VelocityControl(configs.get(i), ios.get(i), RPM.of(IDLE_RPM)));
        }
        register();
        setDefaultCommand(stopCommand());
    }

    @Override
    public String getName() {
        return "Shooter";
    }

    public Command spinUp(Supplier<AngularVelocity> velocity) {
        return run(() -> goalRpm = velocity.get().in(RPM)).withName("Shooter.SpinUp");
    }

    public Command spinUpToDefault() {
        return spinUp(() -> RPM.of(defaultRpm.get())).withName("Shooter.SpinUpToDefault");
    }

    public Command stopCommand() {
        return run(() -> goalRpm = 0.0).ignoringDisable(true).withName("Shooter.Stop");
    }

    public Command trimUp() {
        return Commands.runOnce(() -> adjustTrim(TRIM_STEP_RPM)).withName("Shooter.TrimUp");
    }

    public Command trimDown() {
        return Commands.runOnce(() -> adjustTrim(-TRIM_STEP_RPM)).withName("Shooter.TrimDown");
    }

    public Command setLowPowerMode(boolean enabled) {
        return Commands.runOnce(() -> lowPowerMode = enabled).withName("Shooter.LowPowerMode");
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::isReadyToFire).withName("Shooter.WaitUntilReady");
    }

    public void setGoal(AngularVelocity velocity) {
        goalRpm = velocity.in(RPM);
    }

    public boolean isReadyToFire() {
        return atGoal && goalRpm > IDLE_RPM;
    }

    public double goalRpm() {
        return goalRpm;
    }

    public double trimmedGoalRpm() {
        return goalRpm == 0.0 ? 0.0 : goalRpm + trimRpm;
    }

    public double velocityRpm() {
        if (active.isEmpty()) {
            return 0.0;
        }
        return active.stream().mapToDouble(m -> m.velocity().in(RPM)).average().orElse(0.0);
    }

    @Override
    public void updateInputs() {
        motors.forEach(VelocityControl::updateInputs);
        if (savedTrimRpm.hasChanged(hashCode())) {
            trimRpm = savedTrimRpm.get();
        }
        active = activeMotors();
        double error = Math.abs(trimmedGoalRpm() - velocityRpm());
        atGoal = atGoalDebouncer.calculate(!active.isEmpty() && error <= toleranceRpm.get());
    }

    @Override
    public void applyOutputs() {
        double goal = trimmedGoalRpm();
        for (VelocityControl motor : motors) {
            if (goal != 0.0 && active.contains(motor)) {
                motor.setGoal(RPM.of(goal));
            } else {
                // When stopped, don't use PID
                motor.stop();
            }
            motor.applyOutputs();
        }
        Logger.recordOutput("Shooter/GoalRPM", goalRpm);
        Logger.recordOutput("Shooter/TrimmedGoalRPM", goal);
        Logger.recordOutput("Shooter/VelocityRPM", velocityRpm());
        Logger.recordOutput("Shooter/AtGoal", atGoal);
        Logger.recordOutput("Shooter/ReadyToFire", isReadyToFire());
        Logger.recordOutput("Shooter/LowPowerMode", lowPowerMode);
        Logger.recordOutput("Shooter/TrimRPM", trimRpm);
        Logger.recordOutput("Shooter/ActiveMotors", active.size());
    }

    private void adjustTrim(double deltaRpm) {
        trimRpm += deltaRpm;
        savedTrimRpm.set(trimRpm);
    }

    private List<VelocityControl> activeMotors() {
        if (!lowPowerMode) {
            // Not low power mode, run everything
            return motors.stream().filter(VelocityControl::isConnected).toList();
        }
        // Low power mode, pick one motor, prioritizing the middle motor,
        // but falling back to one of the others if middle is unhealthy.
        for (int index : new int[] {1, 0, 2}) {
            if (motors.get(index).isConnected()) {
                return List.of(motors.get(index));
            }
        }
        return List.of();
    }

    @Override
    public void close() {
        motors.forEach(VelocityControl::close);
    }
}
