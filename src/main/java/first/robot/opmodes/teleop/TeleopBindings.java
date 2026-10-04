package first.robot.opmodes.teleop;

import first.robot.RobotState;
import first.robot.mechanisms.drive.Drive;
import first.robot.mechanisms.drive.JoystickInputs;
import org.wpilib.command3.button.CommandGamepad;

public final class TeleopBindings {
    public TeleopBindings(CommandGamepad driver, CommandGamepad debug, Drive drive, RobotState robotState) {
        drive.setDefaultCommand(drive.joystickDrive(JoystickInputs.from(driver), driver.faceUp()));
        driver.faceLeft().whileTrue(drive.stopWithX());
        driver.start().onTrue(robotState.resetRotationCommand());

        debug.faceDown().whileTrue(drive.wheelRadiusCharacterization());
    }
}
