package first.robot.opmodes.auto;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import first.robot.RobotState;
import first.robot.game.GameCommands;
import first.robot.game.ShotCalculator.PresetDistance;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.JoystickInputs;
import first.robot.subsystems.drive.SwerveConfig;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class Autos {
    public static final List<String> UNREGISTERED = List.of("IntakeDeployRetract");

    public static final List<String> AUTOS =
            List.of("HubToDepoToTower", "NormalBumpAutoLeft", "NormalBumpAutoRight", "NoEventMarkerBumpAuto");

    private static final double ROBOT_MASS_KG = 49.8951607;
    private static final double ROBOT_MOI_KG_M2 = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final double DEFAULT_DRIVE_CURRENT_AMPS = 60.0;

    private static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    private static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);

    private final LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto Choices");

    public Autos(Drive drive, RobotState robotState, SwerveConfig swerve, Optional<GameCommands> game) {
        game.ifPresent(Autos::registerNamedCommands);
        AutoBuilder.configure(
                robotState::getEstimatedPose,
                drive::resetPose,
                drive::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive.runRobotVelocity(speeds),
                new PPHolonomicDriveController(TRANSLATION_PID, ROTATION_PID),
                pathPlannerConfig(swerve),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drive);

        chooser.addDefaultOption("None", Commands.none());
        for (String name : AUTOS) {
            chooser.addOption(name, AutoBuilder.buildAuto(name));
        }
        // Additionally warm up PathPlanner (not sure how necessary it is)
        FollowPathCommand.warmupCommand().schedule();
    }

    public Command selected() {
        return Optional.ofNullable(chooser.get()).orElse(Commands.none());
    }

    private static void registerNamedCommands(GameCommands game) {
        var m = game.mechanisms();
        NamedCommands.registerCommand("IntakeDeployExtend", m.intakeDeploy().extend());
        NamedCommands.registerCommand("CollectorIntake", m.collector().intake());
        NamedCommands.registerCommand("WarmupShooterNear", game.prepareToShoot(PresetDistance.NEAR));
        NamedCommands.registerCommand("FireWhenShooterAndHoodReady", game.fireWhenReady());
        NamedCommands.registerCommand("WaitForRotationAndHoodAndShooterToBeAtGoal", game.waitForShooterHoodAndAim());
        NamedCommands.registerCommand("AimAndShootFromHere", game.aimAndShootFromHere(JoystickInputs.NONE));
    }

    static com.pathplanner.lib.config.RobotConfig pathPlannerConfig(SwerveConfig swerve) {
        var module = swerve.modules().get(0);
        double currentLimit = module.DriveMotorInitialConfigs instanceof TalonFXConfiguration talon
                        && talon.CurrentLimits.StatorCurrentLimitEnable
                ? talon.CurrentLimits.StatorCurrentLimit
                : DEFAULT_DRIVE_CURRENT_AMPS;
        var moduleConfig = new ModuleConfig(
                swerve.wheelRadiusMeters(),
                swerve.maxLinearSpeedMetersPerSec(),
                WHEEL_COF,
                DCMotor.getKrakenX60(1),
                module.DriveMotorGearRatio,
                currentLimit,
                1);
        var offsets = swerve.modules().stream()
                .map(m -> new Translation2d(m.LocationX, m.LocationY))
                .toArray(Translation2d[]::new);
        return new com.pathplanner.lib.config.RobotConfig(ROBOT_MASS_KG, ROBOT_MOI_KG_M2, moduleConfig, offsets);
    }
}
