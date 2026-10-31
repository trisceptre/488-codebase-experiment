package first.robot;

import first.robot.util.LoggedTunableBoolean;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;

public final class RobotState implements LoopParticipant, AutoCloseable {
    public record OdometryObservation(double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

    public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final LoggedTunableNumber visionTimeoutSeconds =
            new LoggedTunableNumber("RobotState/VisionTimeoutSeconds", 5.0);
    private final LoggedTunableBoolean useVisionAssistedPose =
            new LoggedTunableBoolean("RobotState/UseVisionAssistedPose", true);
    private final Alert noVisionAlert =
            new Alert("RobotState/NoVision", "No accepted vision recently; pose is odometry only", Alert.Level.MEDIUM);
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;

    public RobotState(SwerveDriveKinematics kinematics, int moduleCount) {
        var zero = new SwerveModulePosition[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            zero[i] = new SwerveModulePosition();
        }
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.ZERO, zero, Pose2d.ZERO);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.ZERO, zero, Pose2d.ZERO);
    }

    public void addOdometryObservation(OdometryObservation observation) {
        poseEstimator.updateWithTime(observation.timestamp(), observation.yaw(), observation.wheelPositions());
        odometry.update(observation.yaw(), observation.wheelPositions());
    }

    public void addVisionObservation(VisionObservation observation) {
        poseEstimator.addVisionMeasurement(
                observation.visionPose().toPose2d(), observation.timestamp(), observation.stdDevs());
        lastVisionTimestamp = Math.max(lastVisionTimestamp, observation.timestamp());
    }

    public Pose2d getEstimatedPose() {
        return useVisionAssistedPose.get() ? poseEstimator.getEstimatedPosition() : getOdometryPose();
    }

    public Pose2d getOdometryPose() {
        return odometry.getPose();
    }

    public Rotation2d getRotation() {
        return getEstimatedPose().getRotation();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
        odometry.resetPose(pose);
    }

    public void resetRotation(Rotation2d rotation) {
        resetPose(new Pose2d(getEstimatedPose().getTranslation(), rotation));
    }

    public Command resetRotationCommand() {
        return Command.noRequirements(_ -> {
                    boolean red =
                            MatchState.getAlliance().map(a -> a == Alliance.RED).orElse(false);
                    resetRotation(red ? Rotation2d.k180deg : Rotation2d.ZERO);
                })
                .named("RobotState.ResetRotation");
    }

    public double getSecondsSinceVision() {
        return Timer.getTimestamp() - lastVisionTimestamp;
    }

    @Override
    public void updateInputs() {
        Logger.recordOutput("RobotState/EstimatedPose", getEstimatedPose());
        Logger.recordOutput("RobotState/OdometryPose", getOdometryPose());
        Logger.recordOutput("RobotState/SecondsSinceVision", Math.min(getSecondsSinceVision(), 1e6));
        noVisionAlert.set(org.wpilib.driverstation.RobotState.isEnabled()
                && getSecondsSinceVision() > visionTimeoutSeconds.get());
    }

    @Override
    public void applyOutputs() {}

    @Override
    public void close() {
        noVisionAlert.close();
    }
}
