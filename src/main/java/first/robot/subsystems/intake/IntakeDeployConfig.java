package first.robot.subsystems.intake;

import first.robot.util.motor.CANId;

public record IntakeDeployConfig(
        CANId motor,
        CANId encoder,
        boolean inverted,
        double kP,
        double peakVolts,
        double cruiseRotationsPerSec,
        double accelRotationsPerSecSq,
        double rampSeconds,
        double statorAmps,
        double supplyAmps,
        double supplyLowerAmps,
        double supplyLowerSeconds) {}
