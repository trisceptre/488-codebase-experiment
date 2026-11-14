package first.lib.motor;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.system.DCMotor;

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
        Shaping shaping,
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
        Objects.requireNonNull(shaping, "shaping");
        Objects.requireNonNull(simModel, "simModel");
    }

    public record Follower(CANId id, boolean opposeLeader) {}

    public enum GravityType {
        NONE,
        ELEVATOR,
        ARM
    }

    /**
     * Gains are in Phoenix units (volts per mechanism rotation, rotation per second, and so on), while goals, soft
     * limits and Motion Magic constraints are in radians.
     */
    public record Gains(double kS, double kV, double kA, double kG, double kP, double kD, GravityType gravityType) {}

    public record CurrentLimits(double supplyAmps, double statorAmps, Optional<SupplyLowerLimit> supplyLower) {
        public CurrentLimits {
            Objects.requireNonNull(supplyLower, "supplyLower");
        }

        public CurrentLimits(double supplyAmps, double statorAmps) {
            this(supplyAmps, statorAmps, Optional.empty());
        }
    }

    public record SupplyLowerLimit(double amps, double seconds) {}

    public record Shaping(double rampSeconds, Optional<MotionMagic> motionMagic) {
        public static final Shaping NONE = new Shaping(0.0, Optional.empty());

        public Shaping {
            if (!(rampSeconds >= 0.0 && rampSeconds <= 1.0)) {
                throw new IllegalArgumentException("Ramp must be between 0 and 1 second");
            }
        }

        public static Shaping ramp(double seconds) {
            return new Shaping(seconds, Optional.empty());
        }

        public static Shaping motionMagic(double cruiseRadPerSec, double accelRadPerSecSq) {
            return new Shaping(0.0, Optional.of(new MotionMagic(cruiseRadPerSec, accelRadPerSecSq)));
        }
    }

    public record MotionMagic(double cruiseRadPerSec, double accelRadPerSecSq) {
        public MotionMagic {
            if (!(cruiseRadPerSec > 0.0) || !(accelRadPerSecSq > 0.0)) {
                throw new IllegalArgumentException("Motion Magic cruise and acceleration must be > 0");
            }
        }
    }

    public record SoftLimits(double minRad, double maxRad) {
        public SoftLimits {
            if (!(minRad < maxRad)) {
                throw new IllegalArgumentException("minRad must be < maxRad");
            }
        }

        public double clamp(double rad) {
            return Math.clamp(rad, minRad, maxRad);
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
