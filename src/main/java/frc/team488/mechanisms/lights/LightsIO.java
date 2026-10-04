package frc.team488.mechanisms.lights;

import org.littletonrobotics.junction.AutoLog;

/** LED hardware. Default methods are no-ops (sim and replay). */
public interface LightsIO {
    @AutoLog
    class LightsIOInputs {
        public boolean connected = false;
    }

    default void updateInputs(LightsIOInputs inputs) {}

    default void apply(LightPattern pattern) {}
}
