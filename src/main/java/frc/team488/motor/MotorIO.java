package frc.team488.motor;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for one motor-driven mechanism (a leader plus optional followers). All values
 * are in mechanism units: radians and radians/s. Default methods are no-ops, which is the IO used
 * during log replay.
 */
public interface MotorIO {
    @AutoLog
    class MotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyAmps = 0.0;
        public double statorAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean[] followerConnected = new boolean[] {};
        public double[] followerSupplyAmps = new double[] {};
        public double[] followerTempCelsius = new double[] {};
    }

    enum Mode {
        NEUTRAL,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    /**
     * The complete output request for one loop. {@code value} is volts (VOLTAGE), radians
     * (POSITION) or radians/s (VELOCITY). {@code feedforwardVolts} is added to the closed-loop
     * output.
     */
    final class Outputs {
        public Mode mode = Mode.NEUTRAL;
        public double value = 0.0;
        public double feedforwardVolts = 0.0;

        public void setNeutral() {
            mode = Mode.NEUTRAL;
            value = 0.0;
            feedforwardVolts = 0.0;
        }
    }

    default void updateInputs(MotorIOInputs inputs) {}

    default void applyOutputs(Outputs outputs) {}
}
