package first.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Encapsulates the logic needed to rotate the robot to a specific angle.
 * When using a PD or PID controller, it will attempt to reach that angle.
 * When using a D-only controller, it will just resist any rotational motion.
 */
public final class HeadingModule {
    private static final double AT_HEADING_DEBOUNCE_SECONDS = 0.1;

    private final PIDController pid;
    private final double maxOmegaRadPerSec;
    private Debouncer atHeadingDebouncer = newDebouncer();
    private double toleranceRad = Math.toRadians(2.0);
    private boolean atHeading = false;

    public HeadingModule(double kP, double kD, double maxOmegaRadPerSec) {
        this.pid = new PIDController(kP, 0.0, kD);
        // PID works great, but it assumes there is a linear relationship between your current state and
        // your target state. Since rotation is circular, that's not the case: if you are at 170 degrees,
        // and you want to go to -170 degrees, you could travel -340 degrees... or just +20.
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

    public double calculate(Rotation2d current, Rotation2d target) {
        double errorRad = target.minus(current).getRadians();
        atHeading = atHeadingDebouncer.calculate(Math.abs(errorRad) <= toleranceRad);
        double omega = pid.calculate(current.getRadians(), target.getRadians());
        return MathUtil.clamp(omega, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    }

    public boolean atSetpoint() {
        return atHeading;
    }

    public void reset() {
        pid.reset();
        atHeadingDebouncer = newDebouncer();
        atHeading = false;
    }

    private static Debouncer newDebouncer() {
        return new Debouncer(AT_HEADING_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
    }
}
