package first.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import first.robot.util.PhoenixUtil;
import java.util.ArrayDeque;

public class DriveIOCTRE implements DriveIO, AutoCloseable {
    private static final int QUEUE_CAPACITY = 64;
    private static final int CONFIG_ATTEMPTS = 3;
    private static final double CONFIG_TIMEOUT_SECONDS = 0.25;

    private record Sample(double ctreTimestamp, double yawRad, double[] distances, double[] angles) {}

    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final int moduleCount;
    private final ArrayDeque<Sample> queue = new ArrayDeque<>(QUEUE_CAPACITY);
    private long dropped = 0;

    private final SwerveRequest.ApplyRobotSpeeds velocityRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public DriveIOCTRE(SwerveConfig config) {
        moduleCount = config.moduleCount();
        drivetrain = new SwerveDrivetrain<>(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.drivetrain(),
                config.modules().toArray(new SwerveModuleConstants<?, ?, ?>[0]));
        drivetrain.registerTelemetry(this::onTelemetry);
    }

    public static boolean useOffsetsStoredOnEncoders(SwerveConfig config) {
        var bus = new CANBus(config.drivetrain().CANBusName);
        boolean allRead = true;
        for (SwerveModuleConstants<?, ?, ?> module : config.modules()) {
            try (var encoder = new CANcoder(module.EncoderId, bus)) {
                var magnet = new MagnetSensorConfigs();
                StatusCode status = PhoenixUtil.tryUntilOk(
                        CONFIG_ATTEMPTS, () -> encoder.getConfigurator().refresh(magnet, CONFIG_TIMEOUT_SECONDS));
                if (status.isOK()) {
                    module.withEncoderOffset(magnet.MagnetOffset);
                } else {
                    allRead = false;
                }
            }
        }
        return allRead;
    }

    private void onTelemetry(SwerveDriveState state) {
        var distances = new double[moduleCount];
        var angles = new double[moduleCount];
        for (int j = 0; j < moduleCount; j++) {
            distances[j] = state.ModulePositions[j].distanceMeters;
            angles[j] = state.ModulePositions[j].angle.getRadians();
        }
        var sample = new Sample(state.Timestamp, state.RawHeading.getRadians(), distances, angles);
        synchronized (queue) {
            if (queue.size() >= QUEUE_CAPACITY) {
                queue.pollFirst();
                dropped++;
            }
            queue.addLast(sample);
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        Sample[] samples;
        synchronized (queue) {
            samples = queue.toArray(new Sample[0]);
            queue.clear();
            inputs.droppedOdometrySamples = dropped;
        }
        double ctreToWpilib = Timer.getTimestamp() - Utils.getCurrentTimeSeconds();
        int n = samples.length;
        inputs.odometryTimestamps = new double[n];
        inputs.odometryYawRad = new double[n];
        inputs.odometryModuleDistancesMeters = new double[n * moduleCount];
        inputs.odometryModuleAnglesRad = new double[n * moduleCount];
        for (int i = 0; i < n; i++) {
            inputs.odometryTimestamps[i] = samples[i].ctreTimestamp() + ctreToWpilib;
            inputs.odometryYawRad[i] = samples[i].yawRad();
            System.arraycopy(
                    samples[i].distances(), 0, inputs.odometryModuleDistancesMeters, i * moduleCount, moduleCount);
            System.arraycopy(samples[i].angles(), 0, inputs.odometryModuleAnglesRad, i * moduleCount, moduleCount);
        }

        var state = drivetrain.getState();
        inputs.moduleSpeedsMetersPerSec = new double[moduleCount];
        inputs.moduleAnglesRad = new double[moduleCount];
        inputs.driveConnected = new boolean[moduleCount];
        inputs.steerConnected = new boolean[moduleCount];
        inputs.encoderConnected = new boolean[moduleCount];
        for (int j = 0; j < moduleCount; j++) {
            if (state.ModuleStates != null) {
                inputs.moduleSpeedsMetersPerSec[j] = state.ModuleStates[j].speedMetersPerSecond;
                inputs.moduleAnglesRad[j] = state.ModuleStates[j].angle.getRadians();
            }
            var module = drivetrain.getModule(j);
            inputs.driveConnected[j] = module.getDriveMotor().isConnected();
            inputs.steerConnected[j] = module.getSteerMotor().isConnected();
            inputs.encoderConnected[j] = module.getEncoder().isConnected();
        }
        inputs.gyroConnected = drivetrain.getPigeon2().isConnected();
    }

    @Override
    public void setRobotVelocity(ChassisSpeeds velocity) {
        velocityRequest.Speeds = velocity;
        drivetrain.setControl(velocityRequest);
    }

    @Override
    public void stopWithX() {
        drivetrain.setControl(brakeRequest);
    }

    @Override
    public void idle() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void close() {
        drivetrain.close();
    }
}
