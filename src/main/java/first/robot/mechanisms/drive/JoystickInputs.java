package first.robot.mechanisms.drive;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public record JoystickInputs(
        DoubleSupplier leftX,
        DoubleSupplier leftY,
        DoubleSupplier leftTrigger,
        DoubleSupplier rightTrigger,
        DoubleSupplier rightX,
        DoubleSupplier rightY) {

    public static JoystickInputs from(CommandXboxController gamepad) {
        return new JoystickInputs(
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getLeftTriggerAxis,
                gamepad::getRightTriggerAxis,
                gamepad::getRightX,
                gamepad::getRightY);
    }
}
