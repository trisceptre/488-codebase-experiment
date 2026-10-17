package first.robot.opmodes.teleop;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import first.robot.RobotState;
import first.robot.mechanisms.drive.Drive;
import first.robot.mechanisms.drive.JoystickInputs;

public final class TeleopBindings {
    public TeleopBindings(
            CommandXboxController driver, CommandXboxController debug, Drive drive, RobotState robotState) {
        drive.setDefaultCommand(drive.joystickDrive(JoystickInputs.from(driver), driver.y()));
        driver.x().whileTrue(drive.stopWithX());
        driver.start().onTrue(robotState.resetRotationCommand());

        debug.a().whileTrue(drive.wheelRadiusCharacterization());
    }
}
