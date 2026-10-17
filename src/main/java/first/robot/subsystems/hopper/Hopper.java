package first.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.VelocityControl;

public final class Hopper implements Subsystem, LoopParticipant, AutoCloseable {
    private final VelocityControl motor;
    private final LoggedTunableNumber collectRpm = new LoggedTunableNumber("Hopper/CollectRPM", 3000.0);
    private final LoggedTunableNumber intakeRpm = new LoggedTunableNumber("Hopper/IntakeRPM", 3000.0);
    private final LoggedTunableNumber ejectRpm = new LoggedTunableNumber("Hopper/EjectRPM", -3000.0);

    public Hopper(MotorConfig config, MotorIO io) {
        motor = new VelocityControl(config, io, RPM.of(200));
        register();
        setDefaultCommand(stopCommand());
    }

    @Override
    public String getName() {
        return "Hopper";
    }

    public Command collect() {
        return run(() -> motor.setGoal(RPM.of(collectRpm.get()))).withName("Hopper.Collect");
    }

    public Command intake() {
        return run(() -> motor.setGoal(RPM.of(intakeRpm.get()))).withName("Hopper.Intake");
    }

    public Command eject() {
        return run(() -> motor.setGoal(RPM.of(ejectRpm.get()))).withName("Hopper.Eject");
    }

    public Command stopCommand() {
        return run(motor::stop).withName("Hopper.Stop");
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
