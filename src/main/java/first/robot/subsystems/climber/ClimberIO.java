package first.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean atRetractedLimit = false;
    }

    default void updateInputs(ClimberIOInputs inputs) {}
}
