package first.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.lib.LoopParticipant;
import first.lib.motor.MotorConfig;
import first.lib.motor.MotorIO;
import first.lib.motor.PositionControl;
import first.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public final class Climber implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double NOMINAL_VOLTS = 12.0;

    private final PositionControl motors;
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final LoggedTunableNumber extendPower = new LoggedTunableNumber("Climber/ExtendPower", 0.2);
    private final LoggedTunableNumber retractPower = new LoggedTunableNumber("Climber/RetractPower", -0.2);
    private final LoggedTunableNumber extendedDeg = new LoggedTunableNumber("Climber/ExtendedDeg", 180.0);
    private final LoggedTunableNumber engagedDeg = new LoggedTunableNumber("Climber/EngagedDeg", 85.0);
    private final LoggedTunableNumber readinessTimeout =
            new LoggedTunableNumber("Climber/ReadinessTimeoutSeconds", 3.0);

    private boolean calibrated = false;
    private double goalDeg = 0.0;

    public Climber(MotorConfig config, MotorIO motorIO, ClimberIO io) {
        motors = new PositionControl(config, motorIO, Degrees.of(3.0));
        this.io = io;
        register();
        setDefaultCommand(holdCommand());
    }

    @Override
    public String getName() {
        return "Climber";
    }

    public Command holdCommand() {
        return run(this::holdGoal).withName("Climber.Hold");
    }

    public Command extendOpenLoop() {
        return run(() -> motors.runVoltage(extendPower.get() * NOMINAL_VOLTS)).withName("Climber.ExtendOpenLoop");
    }

    public Command retractOpenLoop() {
        return run(() -> {
                    double volts = retractPower.get() * NOMINAL_VOLTS;
                    if (calibrated) {
                        motors.runVoltage(volts);
                    } else {
                        motors.runCalibrationVoltage(volts);
                    }
                })
                .withName("Climber.RetractOpenLoop");
    }

    public Command goTo(double degrees) {
        return runOnce(() -> goalDeg = degrees).andThen(holdCommand()).withName("Climber.GoTo");
    }

    public Command extend() {
        return goTo(extendedDeg.get()).withName("Climber.Extend");
    }

    public Command engage() {
        return goTo(engagedDeg.get()).withName("Climber.Engage");
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(motors::atGoal).withTimeout(readinessTimeout.get());
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public double angleDeg() {
        return motors.position().in(Degrees);
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        motors.updateInputs();
        if (inputs.atRetractedLimit && !calibrated) {
            calibrated = motors.seedPosition(Degrees.of(0.0));
            if (calibrated) {
                goalDeg = 0.0;
            }
        }
    }

    @Override
    public void applyOutputs() {
        motors.applyOutputs();
        Logger.recordOutput("Climber/Calibrated", calibrated);
        Logger.recordOutput("Climber/AngleDeg", angleDeg());
        Logger.recordOutput("Climber/GoalDeg", goalDeg);
    }

    private void holdGoal() {
        if (calibrated) {
            motors.setGoal(Degrees.of(goalDeg));
        } else {
            motors.stop();
        }
    }

    @Override
    public void close() {
        motors.close();
    }
}
