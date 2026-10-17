package first.robot.util.motor;

import org.littletonrobotics.junction.AutoLog;

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
        /**
         * Voltage mode is used to control the motor controller's output as a voltage.
         */
        VOLTAGE,
        POSITION,
        VELOCITY
    }

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
