package first.robot.subsystems.lights;

import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
    @AutoLog
    class LightsIOInputs {
        public boolean connected = false;
    }

    default void updateInputs(LightsIOInputs inputs) {}

    default void apply(int segment, LightPattern pattern) {}
}
