package first.robot.subsystems.lights;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class Lights implements Subsystem, LoopParticipant, AutoCloseable {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();
    private final Alert disconnectedAlert = new Alert("Lights controller disconnected", Alert.AlertType.kInfo);
    private final LightPattern[] patterns;
    private final LightPattern[] sent;

    public Lights(LightsIO io, int segmentCount) {
        this.io = io;
        patterns = new LightPattern[segmentCount];
        sent = new LightPattern[segmentCount];
        Arrays.fill(patterns, new LightPattern.Off());
        register();
        setDefaultCommand(idle());
    }

    @Override
    public String getName() {
        return "Lights";
    }

    public int segmentCount() {
        return patterns.length;
    }

    public Command idle() {
        return show(() -> List.of(new LightPattern.Larson(allianceColor()))).withName("Lights.Alliance");
    }

    public Command solid(Color color) {
        return showAll(new LightPattern.Solid(color), "Lights.Solid");
    }

    public Command larson(Color color) {
        return showAll(new LightPattern.Larson(color), "Lights.Larson");
    }

    public Command blink(Color color, double hz) {
        return showAll(new LightPattern.Blink(color, hz), "Lights.Blink");
    }

    public Command show(Supplier<List<LightPattern>> segments) {
        return run(() -> {
                    List<LightPattern> shown = segments.get();
                    for (int i = 0; i < patterns.length; i++) {
                        patterns[i] = shown.get(Math.min(i, shown.size() - 1));
                    }
                })
                .ignoringDisable(true)
                .withName("Lights.Show");
    }

    public LightPattern current(int segment) {
        return patterns[segment];
    }

    private Command showAll(LightPattern shown, String name) {
        return show(() -> List.of(shown)).withName(name);
    }

    public static Color allianceColor() {
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
        for (int i = 0; i < patterns.length; i++) {
            if (!patterns[i].equals(sent[i])) {
                io.apply(i, patterns[i]);
                sent[i] = patterns[i];
            }
            Logger.recordOutput("Lights/Segment" + i, patterns[i].toString());
        }
    }

    @Override
    public void close() {
        disconnectedAlert.close();
    }
}
