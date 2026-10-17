package first.robot.game;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.util.Color;
import first.robot.subsystems.lights.LightPattern;
import first.robot.subsystems.lights.LightPattern.Bounce;
import org.junit.jupiter.api.Test;

class StatusLightsTest {
    private static StatusLights.State state(boolean disabled, boolean cameras, boolean lowVoltage, double hood) {
        return new StatusLights.State(disabled, false, !disabled, cameras, true, lowVoltage, hood, Color.kBlue);
    }

    @Test
    void missingCameraWhileDisabledStrobesRed() {
        var patterns = StatusLights.patterns(state(true, false, false, 0.0));
        assertEquals(new LightPattern.Blink(Color.kRed, 1.0), patterns.get(0));
    }

    @Test
    void lowVoltageInTeleopIsGreenAndNormalIsRed() {
        assertEquals(
                new LightPattern.Larson(Color.kGreen, Bounce.BACK),
                StatusLights.patterns(state(false, true, true, 0.0)).get(0));
        assertEquals(
                new LightPattern.Larson(Color.kFirstRed, Bounce.BACK),
                StatusLights.patterns(state(false, true, false, 0.0)).get(0));
    }

    @Test
    void hoodSegmentWarnsWhenRaisedAndLastSegmentIsAlliance() {
        var patterns = StatusLights.patterns(state(false, true, false, 0.5));
        assertEquals(new LightPattern.Larson(Color.kDarkRed, Bounce.FRONT), patterns.get(1));
        assertEquals(new LightPattern.Larson(Color.kBlue), patterns.get(2));
    }
}
