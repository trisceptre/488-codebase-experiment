package first.robot.opmodes.teleop;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import first.robot.RobotState;
import first.robot.game.GameCommands;
import first.robot.game.ShotCalculator.PresetDistance;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.JoystickInputs;
import java.util.Optional;

public final class TeleopBindings {
    public TeleopBindings(
            CommandXboxController driver,
            CommandXboxController operator,
            CommandXboxController debug,
            Drive drive,
            RobotState robotState,
            Optional<GameCommands> game) {
        var sticks = JoystickInputs.from(driver);
        drive.setDefaultCommand(drive.joystickDrive(sticks, driver.y()));
        driver.x().whileTrue(drive.stopWithX());
        driver.start().onTrue(robotState.resetRotationCommand());

        debug.back().whileTrue(drive.wheelRadiusCharacterization());

        game.ifPresent(g -> bindGame(g, sticks, driver, operator, debug));
    }

    private static void bindGame(
            GameCommands g,
            JoystickInputs sticks,
            CommandXboxController driver,
            CommandXboxController operator,
            CommandXboxController debug) {
        var m = g.mechanisms();

        driver.a().whileTrue(g.rotateToHub(sticks));
        driver.b().whileTrue(g.aimAndShootFromHere(sticks));
        driver.rightBumper().whileTrue(g.intakeSlowlyAndFireWhenReady());

        m.intakeDeploy().setDefaultCommand(m.intakeDeploy().manualControl(() -> -operator.getLeftY()));
        operator.rightTrigger().whileTrue(g.intakeSlowlyAndFireWhenReady());
        operator.leftTrigger().whileTrue(g.collect());
        operator.leftBumper().whileTrue(m.intakeDeploy().retract());
        operator.rightBumper().whileTrue(m.intakeDeploy().extend());
        operator.x().whileTrue(g.prepareToShoot(PresetDistance.NEAR));
        operator.y().whileTrue(g.prepareToShoot(PresetDistance.TRENCH));
        operator.b().whileTrue(g.prepareToShoot(PresetDistance.CORNER));
        operator.a().whileTrue(g.prepareToShoot(PresetDistance.TOWER_FAR));
        operator.povDown().whileTrue(g.ejectAll());

        debug.leftTrigger().onTrue(m.shooter().trimDown());
        debug.rightTrigger().onTrue(m.shooter().trimUp());
        debug.x().whileTrue(m.shooter().spinUpToDefault());
        debug.y().whileTrue(m.collector().intake());
        debug.a().onTrue(m.intakeDeploy().extend());
        debug.b().onTrue(m.intakeDeploy().retract());
        debug.leftStick().whileTrue(m.hopper().eject());
        debug.rightStick().whileTrue(m.hopper().intake());
        debug.povRight().whileTrue(m.collector().eject());
        debug.povLeft().whileTrue(m.feeder().fire());
    }
}
