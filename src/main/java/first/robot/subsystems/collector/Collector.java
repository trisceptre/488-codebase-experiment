package first.robot.subsystems.collector;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.VelocityControl;

public final class Collector implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double NOMINAL_VOLTS = 12.0;

    private final VelocityControl motor;
    private final LoggedTunableNumber intakeRpm = new LoggedTunableNumber("Collector/IntakeRPM", 5000.0);
    private final LoggedTunableNumber whileShootingRpm =
            new LoggedTunableNumber("Collector/IntakeWhileShootingRPM", 1000.0);
    private final LoggedTunableNumber ejectPower = new LoggedTunableNumber("Collector/EjectPower", -1.0);

    public Collector(MotorConfig config, MotorIO io) {
        motor = new VelocityControl(config, io, RPM.of(300));
        register();
        setDefaultCommand(stopCommand());
    }

    @Override
    public String getName() {
        return "Collector";
    }

    public Command intake() {
        return run(() -> motor.setGoal(RPM.of(intakeRpm.get()))).withName("Collector.Intake");
    }

    public Command intakeWhileShooting() {
        return run(() -> motor.setGoal(RPM.of(whileShootingRpm.get()))).withName("Collector.IntakeWhileShooting");
    }

    public Command eject() {
        return run(() -> motor.runVoltage(ejectPower.get() * NOMINAL_VOLTS)).withName("Collector.Eject");
    }

    public Command stopCommand() {
        return run(motor::stop).withName("Collector.Stop");
    }

    public boolean isIntaking() {
        return motor.goal().map(g -> g.in(RPM) > 0.0).orElse(false);
    }

    @Override
    public void updateInputs() {
        motor.updateInputs();
    }

    @Override
    public void applyOutputs() {
        motor.applyOutputs();
    }

    @Override
    public void close() {
        motor.close();
    }
}
