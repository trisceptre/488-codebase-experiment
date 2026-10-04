package frc.team488.motor;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.system.DCMotor;

/**
 * Everything needed to build a motor-driven mechanism: wiring, gearing, gains, limits and the
 * physics model used in simulation. A new mechanism is mostly one of these.
 *
 * @param sensorToMechanismRatio motor rotations per mechanism rotation (greater than 1 is a reduction)
 * @param gains feedback and feedforward gains, in Phoenix units of mechanism rotations
 * @param softLimits optional position limits in mechanism radians
 */
public record MotorConfig(
        String name,
        CanId leader,
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

    /** A motor that mirrors the leader. */
    public record Follower(CanId id, boolean opposeLeader) {}

    /** Which gravity feedforward the mechanism needs. */
    public enum GravityType {
        NONE,
        ELEVATOR,
        /** kG is scaled by cos(position); position 0 rad must be horizontal. */
        ARM
    }

    /**
     * Gains in Phoenix units of mechanism rotations: kP in volts per rotation (position) or per
     * rotation/s (velocity), kD per derivative, kV in volts per rotation/s, kA in volts per
     * rotation/s², kS and kG in volts.
     */
    public record Gains(double kS, double kV, double kA, double kG, double kP, double kD, GravityType gravityType) {}

    public record CurrentLimits(double supplyAmps, double statorAmps) {}

    /** Position limits in mechanism radians. */
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

    /** The physics model {@link MotorIOSim} uses for this mechanism. */
    public sealed interface SimModel {
        DCMotor motor();

        record Flywheel(DCMotor motor, double moiKgM2) implements SimModel {}

        /** Angles are radians with 0 horizontal. */
        record Arm(DCMotor motor, double moiKgM2, double lengthMeters, double minRad, double maxRad)
                implements SimModel {}

        record Elevator(
                DCMotor motor, double carriageMassKg, double drumRadiusMeters, double minMeters, double maxMeters)
                implements SimModel {}
    }
}
