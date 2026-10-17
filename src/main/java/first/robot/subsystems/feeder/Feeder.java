package first.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.VelocityControl;

public final class Feeder implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double NOMINAL_VOLTS = 12.0;

    private final VelocityControl motor;
    private final LoggedTunableNumber firePower = new LoggedTunableNumber("Feeder/FirePower", 1.0);
    private final LoggedTunableNumber shotCurrentAmps = new LoggedTunableNumber("Feeder/ShotCurrentAmps", 10.0);
    private final LoggedTunableNumber settleSeconds = new LoggedTunableNumber("Feeder/ShotSettleSeconds", 0.5);
    private boolean firing = false;

    public Feeder(MotorConfig config, MotorIO io) {
        motor = new VelocityControl(config, io, RPM.of(200));
        register();
        setDefaultCommand(stopCommand());
    }

    @Override
    public String getName() {
        return "Feeder";
    }

    public Command fire() {
        return run(() -> {
                    motor.runVoltage(firePower.get() * NOMINAL_VOLTS);
                    firing = true;
                })
                .withName("Feeder.Fire");
    }

    public Command eject() {
        return run(() -> {
                    motor.runVoltage(-firePower.get() * NOMINAL_VOLTS);
                    firing = false;
                })
                .withName("Feeder.Eject");
    }

    public Command stopCommand() {
        return run(() -> {
                    motor.stop();
                    firing = false;
                })
                .withName("Feeder.Stop");
    }

    public boolean isFiring() {
        return firing;
    }

    public double statorAmps() {
        return motor.statorAmps();
    }

    public Command waitUntilShootingFinished() {
        var shot = new double[] {Double.NaN};
        return Commands.run(() -> {
                    if (statorAmps() > shotCurrentAmps.get()) {
                        shot[0] = Timer.getTimestamp();
                    }
                })
                .beforeStarting(() -> shot[0] = Double.NaN)
                .until(() -> !Double.isNaN(shot[0]) && Timer.getTimestamp() - shot[0] > settleSeconds.get())
                .withName("Feeder.WaitUntilShootingFinished");
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
