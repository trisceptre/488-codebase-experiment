package first.robot.game;

import edu.wpi.first.wpilibj.util.Color;
import first.robot.subsystems.lights.LightPattern;
import first.robot.subsystems.lights.LightPattern.Bounce;
import java.util.List;

public final class StatusLights {
    private StatusLights() {}

    public static final double HOOD_RAISED_RATIO = 0.02;

    public record State(
            boolean disabled,
            boolean autonomous,
            boolean teleop,
            boolean allCamerasConnected,
            boolean intakeCalibrated,
            boolean lowVoltage,
            double hoodRatio,
            Color allianceColor) {}

    public static List<LightPattern> patterns(State state) {
        return List.of(health(state), hood(state), new LightPattern.Larson(state.allianceColor()));
    }

    private static LightPattern health(State state) {
        if (state.disabled() && !state.allCamerasConnected()) {
            return new LightPattern.Blink(Color.kRed, 1.0);
        }
        if (state.intakeCalibrated() && state.lowVoltage() && state.autonomous()) {
            return new LightPattern.Larson(Color.kDodgerBlue, Bounce.BACK);
        }
        if (state.intakeCalibrated() && state.lowVoltage() && state.teleop()) {
            return new LightPattern.Larson(Color.kGreen, Bounce.BACK);
        }
        return new LightPattern.Larson(Color.kFirstRed, Bounce.BACK);
    }

    private static LightPattern hood(State state) {
        return new LightPattern.Larson(
                state.hoodRatio() >= HOOD_RAISED_RATIO ? Color.kDarkRed : Color.kLightGreen, Bounce.FRONT);
    }
}
