package first.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public final class RobotState implements LoopParticipant, AutoCloseable {
    public record OdometryObservation(double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

    public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final LoggedTunableNumber visionTimeoutSeconds =
            new LoggedTunableNumber("RobotState/VisionTimeoutSeconds", 5.0);
    private final Alert noVisionAlert =
            new Alert("No accepted vision recently; pose is odometry only", Alert.AlertType.kWarning);
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;

    public RobotState(SwerveDriveKinematics kinematics, int moduleCount) {
        var zero = new SwerveModulePosition[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            zero[i] = new SwerveModulePosition();
        }
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, zero, Pose2d.kZero);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.kZero, zero, Pose2d.kZero);
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
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
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
        return Commands.runOnce(() -> {
                    boolean red = DriverStation.getAlliance()
                            .map(a -> a == Alliance.Red)
                            .orElse(false);
                    resetRotation(red ? Rotation2d.k180deg : Rotation2d.kZero);
                })
                .ignoringDisable(true)
                .withName("RobotState.ResetRotation");
    }

    public double getSecondsSinceVision() {
        return Timer.getTimestamp() - lastVisionTimestamp;
    }

    @Override
    public void updateInputs() {
        Logger.recordOutput("RobotState/EstimatedPose", getEstimatedPose());
        Logger.recordOutput("RobotState/OdometryPose", getOdometryPose());
        Logger.recordOutput("RobotState/SecondsSinceVision", Math.min(getSecondsSinceVision(), 1e6));
        noVisionAlert.set(
                edu.wpi.first.wpilibj.RobotState.isEnabled() && getSecondsSinceVision() > visionTimeoutSeconds.get());
    }

    @Override
    public void applyOutputs() {}

    @Override
    public void close() {
        noVisionAlert.close();
    }
}
