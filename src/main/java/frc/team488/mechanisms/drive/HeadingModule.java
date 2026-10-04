package frc.team488.mechanisms.drive;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;

/** Turns the robot toward a target heading. Shared by right-stick heading control and aiming. */
public final class HeadingModule {
    private static final double AT_HEADING_DEBOUNCE_SECONDS = 0.1;

    private final PIDController pid;
    private final double maxOmegaRadPerSec;
    private Debouncer atHeadingDebouncer = newDebouncer();
    private double toleranceRad = Math.toRadians(2.0);
    private boolean atHeading = false;

    public HeadingModule(double kP, double kD, double maxOmegaRadPerSec) {
        this.pid = new PIDController(kP, 0.0, kD);
        this.pid.enableContinuousInput(-Math.PI, Math.PI);
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;
    }

    public void setGains(double kP, double kD) {
        pid.setP(kP);
        pid.setD(kD);
    }

    public void setTolerance(Rotation2d tolerance) {
        toleranceRad = Math.abs(tolerance.getRadians());
    }

    /** Angular velocity (rad/s, counter-clockwise positive) to turn from {@code current} to {@code target}. */
    public double calculate(Rotation2d current, Rotation2d target) {
        double errorRad = target.minus(current).getRadians();
        atHeading = atHeadingDebouncer.calculate(Math.abs(errorRad) <= toleranceRad);
        double omega = pid.calculate(current.getRadians(), target.getRadians());
        return Math.clamp(omega, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    }

    /** True once within tolerance for 0.1 s. */
    public boolean atHeading() {
        return atHeading;
    }

    /** Call when heading control stops, so the next use starts fresh. */
    public void reset() {
        pid.reset();
        atHeadingDebouncer = newDebouncer();
        atHeading = false;
    }

    private static Debouncer newDebouncer() {
        return new Debouncer(AT_HEADING_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);
    }
}
