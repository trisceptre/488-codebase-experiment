package frc.team488.mechanisms.drive;

import java.util.Optional;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Turns gamepad axes into a field-relative drive intent using 488's control scheme: left stick
 * translates (squared), triggers rotate, and the right stick, pushed past a threshold, picks a
 * heading to face. Pure functions, so they're unit-tested without hardware.
 */
public final class JoystickShaping {
    private JoystickShaping() {}

    public record Settings(
            double translationDeadband,
            double rotationDeadband,
            double headingStickThreshold,
            double precisionTranslationScale,
            double precisionRotationScale) {
        public static final Settings DEFAULTS = new Settings(0.15, 0.05, 0.5, 0.1, 0.2);
    }

    /**
     * Fractions of max speed in the field frame (origin on the blue wall), and an optional heading
     * the robot should face.
     */
    public record Intent(double vxFraction, double vyFraction, double omegaFraction, Optional<Rotation2d> heading) {}

    /** Zero inside the deadband; rescaled so the output still spans -1..1 outside it. */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    public static Intent shape(
            double leftX,
            double leftY,
            double leftTrigger,
            double rightTrigger,
            double rightX,
            double rightY,
            boolean redAlliance,
            boolean precision,
            Settings settings) {
        // Stick up (negative Y) drives away from the driver's wall; stick left (negative X) drives left.
        double forward = squared(deadband(-leftY, settings.translationDeadband()));
        double left = squared(deadband(-leftX, settings.translationDeadband()));
        double counterClockwise = deadband(leftTrigger, settings.rotationDeadband())
                - deadband(rightTrigger, settings.rotationDeadband());

        double alliance = redAlliance ? -1.0 : 1.0;
        double translationScale = precision ? settings.precisionTranslationScale() : 1.0;
        double rotationScale = precision ? settings.precisionRotationScale() : 1.0;

        Optional<Rotation2d> heading = Optional.empty();
        if (Math.hypot(rightX, rightY) > settings.headingStickThreshold()) {
            Rotation2d stick = new Rotation2d(-rightY, -rightX);
            heading = Optional.of(redAlliance ? stick.plus(Rotation2d.k180deg) : stick);
        }
        return new Intent(
                alliance * forward * translationScale,
                alliance * left * translationScale,
                counterClockwise * rotationScale,
                heading);
    }

    private static double squared(double value) {
        return value * Math.abs(value);
    }
}
