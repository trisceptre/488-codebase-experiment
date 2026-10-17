package first.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import first.robot.game.GameMechanisms;
import first.robot.subsystems.drive.SwerveConfig;
import first.robot.util.motor.MotorIOSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

public final class FuelSim {
    private static final String FUEL = "Fuel";
    private static final int CAPACITY = 30;
    private static final int PRELOADED = 8;
    private static final double SHOTS_PER_SECOND = 10.0;
    private static final double DT = MotorIOSim.LOOP_PERIOD_SECONDS;
    private static final double FLYWHEEL_RADIUS_METERS = 0.025;
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.25, 0.0);
    private static final double LAUNCH_DEG_HOOD_DOWN = 78.0;
    private static final double LAUNCH_DEG_PER_HOOD_RATIO = -24.0;

    private final GameMechanisms mechanisms;
    private final Supplier<Pose2d> truePose;
    private final Arena2026Rebuilt arena;
    private final SwerveDriveSimulation body;
    private final IntakeSimulation intake;
    private Pose2d lastPose;
    private double shotTimer = 0.0;

    public FuelSim(SwerveConfig drive, GameMechanisms mechanisms, Supplier<Pose2d> truePose) {
        this.mechanisms = mechanisms;
        this.truePose = truePose;
        arena = new Arena2026Rebuilt(false);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.overrideSimulationTimings(Seconds.of(DT), 5);

        var modules = drive.modules().stream()
                .map(m -> new Translation2d(m.LocationX, m.LocationY))
                .toArray(Translation2d[]::new);
        var config = DriveTrainSimulationConfig.Default()
                .withBumperSize(Meters.of(0.76), Meters.of(0.76))
                .withCustomModuleTranslations(modules);
        lastPose = truePose.get();
        body = new SwerveDriveSimulation(config, lastPose);
        arena.addDriveTrainSimulation(body);
        intake = IntakeSimulation.OverTheBumperIntake(
                FUEL, body, Inches.of(28), Inches.of(12), IntakeSimulation.IntakeSide.FRONT, CAPACITY);
        reset();
    }

    public void reset() {
        arena.resetFieldForAuto();
        intake.setGamePiecesCount(PRELOADED);
        shotTimer = 0.0;
    }

    public int heldFuel() {
        return intake.getGamePiecesAmount();
    }

    public void update() {
        Pose2d pose = truePose.get();
        var fieldSpeeds = new ChassisSpeeds(
                (pose.getX() - lastPose.getX()) / DT,
                (pose.getY() - lastPose.getY()) / DT,
                pose.getRotation().minus(lastPose.getRotation()).getRadians() / DT);
        lastPose = pose;
        body.setSimulationWorldPose(pose);
        body.setRobotSpeeds(fieldSpeeds);

        boolean intaking =
                mechanisms.intakeDeploy().isDeployed() && mechanisms.collector().isIntaking();
        if (intaking) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }
        arena.simulationPeriodic();
        shoot(pose);

        Logger.recordOutput("FuelSim/Fuel", arena.getGamePiecesArrayByType(FUEL));
        Logger.recordOutput("FuelSim/HeldFuel", heldFuel());
        Logger.recordOutput("FuelSim/Intaking", intaking);
    }

    private void shoot(Pose2d pose) {
        if (!mechanisms.feeder().isFiring()) {
            shotTimer = 0.0;
            return;
        }
        shotTimer += DT;
        while (shotTimer >= 1.0 / SHOTS_PER_SECOND) {
            shotTimer -= 1.0 / SHOTS_PER_SECOND;
            if (!intake.obtainGamePieceFromIntake()) {
                return;
            }
            double wheelRadPerSec = RPM.of(mechanisms.shooter().velocityRpm()).in(RadiansPerSecond);
            double launchDeg = LAUNCH_DEG_HOOD_DOWN
                    + LAUNCH_DEG_PER_HOOD_RATIO * mechanisms.hood().estimatedRatio();
            arena.addPieceWithVariance(
                    pose.getTranslation().plus(SHOOTER_OFFSET.rotateBy(pose.getRotation())),
                    pose.getRotation().plus(Rotation2d.k180deg),
                    Inches.of(20),
                    MetersPerSecond.of(wheelRadPerSec * FLYWHEEL_RADIUS_METERS),
                    Degrees.of(launchDeg),
                    0.1,
                    0.5,
                    2.0,
                    1.0,
                    2.0);
        }
    }
}
