package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
    @AutoLog
    class IntakeDeployIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorAmps = 0.0;
        public double supplyAmps = 0.0;
    }

    default void updateInputs(IntakeDeployIOInputs inputs) {}

    default void setPositionGoal(double positionRad) {}

    default void setVoltage(double volts) {}

    default void stop() {}
}
