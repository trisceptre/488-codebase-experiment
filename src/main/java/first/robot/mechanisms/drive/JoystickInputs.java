package first.robot.mechanisms.drive;

import java.util.function.DoubleSupplier;
import org.wpilib.command3.button.CommandGamepad;

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
