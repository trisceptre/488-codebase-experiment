package first.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.PositionControl;
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
    private double offsetRad = 0.0;
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
        return run(() -> motors.runVoltage(retractPower.get() * NOMINAL_VOLTS)).withName("Climber.RetractOpenLoop");
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
        return Units.radiansToDegrees(motors.position().in(Radians) - offsetRad);
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        motors.updateInputs();
        if (inputs.atRetractedLimit && !calibrated) {
            offsetRad = motors.position().in(Radians);
            goalDeg = 0.0;
            calibrated = true;
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
            motors.setGoal(Radians.of(Units.degreesToRadians(goalDeg) + offsetRad));
        } else {
            motors.stop();
        }
    }

    @Override
    public void close() {
        motors.close();
    }
}
