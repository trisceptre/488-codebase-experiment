package first.robot.game;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import first.robot.RobotState;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.JoystickInputs;
import first.robot.util.LoggedTunableNumber;

public final class GameCommands {
    public enum Target {
        HUB,
        ALLIANCE_ZONE
    }

    private final GameMechanisms m;
    private final Drive drive;
    private final RobotState robotState;
    private final AprilTagFieldLayout field;
    private final ShotCalculator shots;
    private final LoggedTunableNumber aimToleranceDeg = new LoggedTunableNumber("Aim/ToleranceDeg", 3.0);

    public GameCommands(
            GameMechanisms mechanisms,
            Drive drive,
            RobotState robotState,
            AprilTagFieldLayout field,
            ShotCalculator shots) {
        this.m = mechanisms;
        this.drive = drive;
        this.robotState = robotState;
        this.field = field;
        this.shots = shots;
    }

    public GameMechanisms mechanisms() {
        return m;
    }

    public Command prepareToShoot(ShotCalculator.PresetDistance preset) {
        return Commands.parallel(
                        m.shooter().spinUp(() -> shots.preset(preset).shooterSpeed()),
                        m.hood().hold(() -> shots.preset(preset).hoodRatio()))
                .withName("PrepareToShoot." + preset);
    }

    public Command prepareToShootFromHere(Target target, boolean zeroHood) {
        return Commands.run(
                        () -> {
                            var pose = robotState.getEstimatedPose();
                            var shot = target == Target.HUB
                                    ? shots.hubShot(pose, zeroHood)
                                    : shots.allianceZoneShot(pose, zeroHood);
                            m.shooter().setGoal(shot.shooterSpeed());
                            m.hood().setGoal(shot.hoodRatio());
                        },
                        m.shooter(),
                        m.hood())
                .withName("PrepareToShootFromHere." + target);
    }

    public Command waitForShooterAndHood() {
        return Commands.parallel(m.shooter().waitUntilReady(), m.hood().waitUntilAtGoal())
                .withName("WaitForShooterAndHood");
    }

    public Command waitForShooterHoodAndAim() {
        return Commands.parallel(waitForShooterAndHood(), Commands.waitUntil(this::isFacingHub))
                .withName("WaitForShooterHoodAndAim");
    }

    public Command feed() {
        return Commands.parallel(
                        m.hopper().intake(),
                        m.collector().intakeWhileShooting(),
                        m.feeder().fire())
                .withName("Feed");
    }

    public Command fireWhenReady() {
        return waitForShooterAndHood().andThen(feed()).withName("FireWhenReady");
    }

    public Command intakeSlowlyAndFireWhenReady() {
        return waitForShooterAndHood()
                .andThen(Commands.parallel(feed(), m.intakeDeploy().adaptiveCloseWhileFiring()))
                .withName("IntakeSlowlyAndFireWhenReady");
    }

    public Command intakeSlowlyAndFireWhenAimed() {
        return waitForShooterHoodAndAim()
                .andThen(Commands.parallel(feed(), m.intakeDeploy().adaptiveCloseWhileFiring()))
                .withName("IntakeSlowlyAndFireWhenAimed");
    }

    public Command collect() {
        return Commands.parallel(m.collector().intake(), m.hopper().collect()).withName("Collect");
    }

    public Command ejectAll() {
        return Commands.parallel(
                        m.collector().eject(), m.hopper().eject(), m.feeder().eject())
                .withName("EjectAll");
    }

    public Command rotateToHub(JoystickInputs sticks) {
        return drive.joystickDriveAtAngle(sticks, this::hubHeading).withName("RotateToHub");
    }

    public Command aimAndShootFromHere(JoystickInputs sticks) {
        return Commands.parallel(
                        prepareToShootFromHere(Target.HUB, true), rotateToHub(sticks), intakeSlowlyAndFireWhenAimed())
                .withName("AimAndShootFromHere");
    }

    public Rotation2d hubHeading() {
        var hub = Landmarks.allianceHub(field, alliance()).getTranslation();
        var robot = robotState.getEstimatedPose().getTranslation();
        return hub.minus(robot).getAngle().plus(Rotation2d.k180deg);
    }

    public boolean isFacingHub() {
        double error = hubHeading().minus(robotState.getRotation()).getDegrees();
        return Math.abs(error) <= aimToleranceDeg.get();
    }

    private static Alliance alliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
