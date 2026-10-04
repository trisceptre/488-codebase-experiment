package frc.team488.mechanisms.lights;

import org.wpilib.util.Color;

/** What the LEDs show. */
public sealed interface LightPattern {
    record Off() implements LightPattern {}

    record Solid(Color color) implements LightPattern {}

    record Larson(Color color) implements LightPattern {}

    record Blink(Color color, double hz) implements LightPattern {}
}
