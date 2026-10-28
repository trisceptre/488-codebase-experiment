package first.robot.opmodes.teleop;

import first.robot.Robot;
import first.robot.mechanisms.drive.JoystickInputs;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class DefaultTeleop implements OpMode {
    public DefaultTeleop(Robot robot) {
        var driver = robot.driverController;
        var drive = robot.drive;
        drive.setDefaultCommand(drive.joystickDrive(JoystickInputs.from(driver), driver.faceUp()));
        driver.faceLeft().whileTrue(drive.stopWithX());
        driver.start().onTrue(robot.robotState.resetRotationCommand());

        robot.debugController.faceDown().whileTrue(drive.wheelRadiusCharacterization());
    }
}
