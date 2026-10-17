package first.robot.mechanisms.lights;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import org.littletonrobotics.junction.Logger;

public final class Lights implements Subsystem, LoopParticipant, AutoCloseable {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();
    private final Alert disconnectedAlert = new Alert("Lights controller disconnected", Alert.AlertType.kInfo);
    private LightPattern pattern = new LightPattern.Off();
    private LightPattern sent = null;

    public Lights(LightsIO io) {
        this.io = io;
        register();
        setDefaultCommand(idle());
    }

    @Override
    public String getName() {
        return "Lights";
    }

    public Command idle() {
        return run(() -> pattern = new LightPattern.Larson(allianceColor())).withName("Lights.Alliance");
    }

    public Command solid(Color color) {
        return show(new LightPattern.Solid(color), "Lights.Solid");
    }

    public Command larson(Color color) {
        return show(new LightPattern.Larson(color), "Lights.Larson");
    }

    public Command blink(Color color, double hz) {
        return show(new LightPattern.Blink(color, hz), "Lights.Blink");
    }

    public LightPattern current() {
        return pattern;
    }

    private Command show(LightPattern shown, String name) {
        return run(() -> pattern = shown).withName(name);
    }

    private static Color allianceColor() {
        return DriverStation.getAlliance()
                .map(a -> a == Alliance.Red ? Color.kRed : Color.kBlue)
                .orElse(Color.kWhite);
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Lights", inputs);
    }

    @Override
    public void applyOutputs() {
        if (!pattern.equals(sent)) {
            io.apply(pattern);
            sent = pattern;
        }
        Logger.recordOutput("Lights/Pattern", pattern.toString());
    }

    @Override
    public void close() {
        disconnectedAlert.close();
    }
}
