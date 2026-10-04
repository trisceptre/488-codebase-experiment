package first.robot.mechanisms.drive;

import java.util.Optional;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

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

    public record Intent(double vxFraction, double vyFraction, double omegaFraction, Optional<Rotation2d> heading) {}

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
        // If the magnitude is greater than the deadband, exponentiate it and retain the sign.
        // as an example, if the exponent is 2, then a magnitude of 0.5 will become 0.25. In general,
        // higher exponentiation makes the robot respond slower to small inputs, and makes a sharp(er) jump as you
        // approach full power.
        double forward = MathUtil.copyDirectionPow(MathUtil.applyDeadband(-leftY, settings.translationDeadband()), 2);
        double left = MathUtil.copyDirectionPow(MathUtil.applyDeadband(-leftX, settings.translationDeadband()), 2);
        // Deadband is to prevent buggy joysticks/triggers
        // Merge the two trigger values together in case of conflicts
        // Rotate left = positive, right = negative
        double counterClockwise = MathUtil.applyDeadband(leftTrigger, settings.rotationDeadband())
                - MathUtil.applyDeadband(rightTrigger, settings.rotationDeadband());

        // However, if we are on the red alliance, we need to rotate the intent a lot more.
        double alliance = redAlliance ? -1.0 : 1.0;
        // If we are in a precision mode (used for lining up on delicate targets), slow down dramatically
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
}
