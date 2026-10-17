package first.robot.subsystems.lights;

import edu.wpi.first.wpilibj.util.Color;

public sealed interface LightPattern {
    record Off() implements LightPattern {}

    record Solid(Color color) implements LightPattern {}

    record Larson(Color color) implements LightPattern {}

    record Blink(Color color, double hz) implements LightPattern {}
}
