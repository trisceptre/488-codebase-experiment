package frc.team488.controls;

import frc.team488.Pose;
import frc.team488.mechanisms.drive.Drive;
import frc.team488.mechanisms.drive.JoystickInputs;
import org.wpilib.command3.button.CommandGamepad;

/**
 * Driver bindings. This class only registers triggers and has no periodic logic, so it can move
 * into a {@code @Teleop} OpMode's constructor once AdvantageKit ships LoggedOpModeRobot.
 */
public final class TeleopBindings {
    public TeleopBindings(CommandGamepad driver, Drive drive, Pose pose) {
        drive.setDefaultCommand(drive.joystickDrive(JoystickInputs.from(driver), driver.leftBumper()));
        driver.faceLeft().whileTrue(drive.lockWheels());
        driver.back().onTrue(pose.resetHeadingCommand());
    }
}
