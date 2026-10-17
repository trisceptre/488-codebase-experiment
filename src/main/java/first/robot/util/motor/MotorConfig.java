package first.robot.util.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public record MotorConfig(
        String name,
        CANId leader,
        List<Follower> followers,
        boolean inverted,
        boolean brakeMode,
        double sensorToMechanismRatio,
        Gains gains,
        CurrentLimits currentLimits,
        Optional<SoftLimits> softLimits,
        SimModel simModel) {

    public MotorConfig {
        Objects.requireNonNull(name, "name");
        if (name.isBlank()) {
            throw new IllegalArgumentException("name must not be blank");
        }
        Objects.requireNonNull(leader, "leader");
        followers = List.copyOf(followers);
        if (!(sensorToMechanismRatio > 0.0)) {
            throw new IllegalArgumentException(
                    name + ": sensorToMechanismRatio must be > 0, was " + sensorToMechanismRatio);
        }
        Objects.requireNonNull(gains, "gains");
        Objects.requireNonNull(currentLimits, "currentLimits");
        Objects.requireNonNull(softLimits, "softLimits");
        Objects.requireNonNull(simModel, "simModel");
    }

    public record Follower(CANId id, boolean opposeLeader) {}

    public enum GravityType {
        NONE,
        ELEVATOR,
        ARM
    }

    public record Gains(double kS, double kV, double kA, double kG, double kP, double kD, GravityType gravityType) {}

    public record CurrentLimits(double supplyAmps, double statorAmps) {}

    public record SoftLimits(double minRad, double maxRad) {
        public SoftLimits {
            if (!(minRad < maxRad)) {
                throw new IllegalArgumentException("minRad must be < maxRad");
            }
        }

        public double clamp(double rad) {
            return MathUtil.clamp(rad, minRad, maxRad);
        }
    }

    public sealed interface SimModel {
        DCMotor motor();

        record Flywheel(DCMotor motor, double moiKgM2) implements SimModel {}

        record Arm(DCMotor motor, double moiKgM2, double lengthMeters, double minRad, double maxRad)
                implements SimModel {}

        record Elevator(
                DCMotor motor, double carriageMassKg, double drumRadiusMeters, double minMeters, double maxMeters)
                implements SimModel {}
    }
}
