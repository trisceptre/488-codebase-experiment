package frc.team488;

import frc.team488.util.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;

/**
 * The robot's field pose: swerve odometry samples fused with vision. Runs in the main loop from
 * logged inputs, so AdvantageKit replay re-runs the fusion.
 */
public final class Pose implements LoopParticipant, AutoCloseable {
    private final SwerveDrivePoseEstimator estimator;
    private final TunableNumber visionTimeoutSeconds = new TunableNumber("Pose/VisionTimeoutSeconds", 5.0);
    private final Alert noVisionAlert =
            new Alert("Pose/NoVision", "No accepted vision recently; pose is odometry only", Alert.Level.MEDIUM);
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;

    public Pose(SwerveDriveKinematics kinematics, int moduleCount) {
        var zero = new SwerveModulePosition[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            zero[i] = new SwerveModulePosition();
        }
        estimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.ZERO, zero, Pose2d.ZERO);
    }

    public void addOdometrySample(double timestamp, Rotation2d yaw, SwerveModulePosition[] positions) {
        estimator.updateWithTime(timestamp, yaw, positions);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
        lastVisionTimestamp = Math.max(lastVisionTimestamp, timestamp);
    }

    public Pose2d get() {
        return estimator.getEstimatedPosition();
    }

    public Rotation2d heading() {
        return get().getRotation();
    }

    public void reset(Pose2d pose) {
        estimator.resetPose(pose);
    }

    public void resetHeading(Rotation2d heading) {
        reset(new Pose2d(get().getTranslation(), heading));
    }

    /** Resets heading to face away from the driver's alliance wall. */
    public Command resetHeadingCommand() {
        return Command.noRequirements(_ -> {
                    boolean red =
                            MatchState.getAlliance().map(a -> a == Alliance.RED).orElse(false);
                    resetHeading(red ? Rotation2d.k180deg : Rotation2d.ZERO);
                })
                .named("Pose.ResetHeading");
    }

    public double secondsSinceVision() {
        return Timer.getTimestamp() - lastVisionTimestamp;
    }

    @Override
    public void updateInputs() {
        Logger.recordOutput("Pose/Estimated", get());
        Logger.recordOutput("Pose/SecondsSinceVision", Math.min(secondsSinceVision(), 1e6));
        noVisionAlert.set(RobotState.isEnabled() && secondsSinceVision() > visionTimeoutSeconds.get());
    }

    @Override
    public void applyOutputs() {}

    @Override
    public void close() {
        noVisionAlert.close();
    }
}
