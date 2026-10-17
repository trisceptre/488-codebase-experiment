package first.robot.subsystems.lights;

import edu.wpi.first.wpilibj.util.Color;

public sealed interface LightPattern {
    double DEFAULT_LARSON_HZ = 25.0;

    record Off() implements LightPattern {}

    record Solid(Color color) implements LightPattern {}

    record Larson(Color color, Bounce bounce, double hz) implements LightPattern {
        public Larson(Color color) {
            this(color, Bounce.FRONT, DEFAULT_LARSON_HZ);
        }

        public Larson(Color color, Bounce bounce) {
            this(color, bounce, DEFAULT_LARSON_HZ);
        }
    }

    record Blink(Color color, double hz) implements LightPattern {}

    enum Bounce {
        FRONT,
        BACK
    }
}
