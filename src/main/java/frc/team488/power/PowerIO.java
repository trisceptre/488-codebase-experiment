package frc.team488.power;

import org.littletonrobotics.junction.AutoLog;

/** Battery and power distribution readings. Default is a no-op (replay). */
public interface PowerIO {
    @AutoLog
    class PowerIOInputs {
        public boolean connected = false;
        public double voltage = 0.0;
        public double totalCurrentAmps = 0.0;
    }

    default void updateInputs(PowerIOInputs inputs) {}
}
