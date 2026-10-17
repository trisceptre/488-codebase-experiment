package first.robot.util.power;

import org.littletonrobotics.junction.AutoLog;

public interface PowerIO {
    @AutoLog
    class PowerIOInputs {
        public boolean connected = false;
        public double voltage = 0.0;
        public double totalCurrentAmps = 0.0;
    }

    default void updateInputs(PowerIOInputs inputs) {}
}
