package frc.team488.mechanisms.drive;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.button.CommandGamepad;

/** Raw driver axes, as suppliers so drive commands read fresh values every loop. */
public record JoystickInputs(
        DoubleSupplier leftX,
        DoubleSupplier leftY,
        DoubleSupplier leftTrigger,
        DoubleSupplier rightTrigger,
        DoubleSupplier rightX,
        DoubleSupplier rightY) {

    public static JoystickInputs from(CommandGamepad gamepad) {
        return new JoystickInputs(
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getLeftTrigger,
                gamepad::getRightTrigger,
                gamepad::getRightX,
                gamepad::getRightY);
    }
}
