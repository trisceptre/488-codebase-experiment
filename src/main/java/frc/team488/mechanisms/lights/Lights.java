package frc.team488.mechanisms.lights;

import frc.team488.LoopParticipant;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.util.Alert;
import org.wpilib.util.Color;

/** LEDs. Default: an alliance-colored Larson scanner (white until the alliance is known). */
public final class Lights implements Mechanism, LoopParticipant, AutoCloseable {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();
    private final Alert disconnectedAlert =
            new Alert("Lights/Disconnected", "Lights controller disconnected", Alert.Level.LOW);
    private LightPattern pattern = new LightPattern.Off();
    private LightPattern sent = null;

    public Lights(LightsIO io) {
        this.io = io;
        setDefaultCommand(idle());
    }

    @Override
    public String getName() {
        return "Lights";
    }

    @Override
    public Command idle() {
        return runRepeatedly(() -> pattern = new LightPattern.Larson(allianceColor()))
                .withPriority(Command.LOWEST_PRIORITY)
                .named("Lights.Alliance");
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
        return run(coroutine -> {
                    pattern = shown;
                    coroutine.park();
                })
                .named(name);
    }

    private static Color allianceColor() {
        return MatchState.getAlliance()
                .map(a -> a == Alliance.RED ? Color.RED : Color.BLUE)
                .orElse(Color.WHITE);
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
