package frc.team488.mechanisms.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.ArrayDeque;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * DriveIO on CTRE's {@link SwerveDrivetrain}. CTRE runs the modules and samples odometry on its own
 * thread; each sample is copied into a bounded queue that {@link #updateInputs} drains.
 */
public class DriveIOCtre implements DriveIO, AutoCloseable {
    private static final int QUEUE_CAPACITY = 64;

    private record Sample(double ctreTimestamp, double yawRad, double[] distances, double[] angles) {}

    protected final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final int moduleCount;
    private final ArrayDeque<Sample> queue = new ArrayDeque<>(QUEUE_CAPACITY);
    private long dropped = 0;

    private final SwerveRequest.ApplyRobotVelocity velocityRequest = new SwerveRequest.ApplyRobotVelocity();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public DriveIOCtre(SwerveConfig config) {
        moduleCount = config.moduleCount();
        drivetrain = new SwerveDrivetrain<>(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.drivetrain(),
                config.modules().toArray(new SwerveModuleConstants<?, ?, ?>[0]));
        drivetrain.registerTelemetry(this::onTelemetry);
    }

    private void onTelemetry(SwerveDriveState state) {
        var distances = new double[moduleCount];
        var angles = new double[moduleCount];
        for (int j = 0; j < moduleCount; j++) {
            distances[j] = state.ModulePositions[j].distance;
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
        // CTRE stamps samples with its own clock; shift them onto the WPILib timer.
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
            if (state.ModuleVelocities != null) {
                inputs.moduleSpeedsMetersPerSec[j] = state.ModuleVelocities[j].velocity;
                inputs.moduleAnglesRad[j] = state.ModuleVelocities[j].angle.getRadians();
            }
            var module = drivetrain.getModule(j);
            inputs.driveConnected[j] = module.getDriveMotor().isConnected();
            inputs.steerConnected[j] = module.getSteerMotor().isConnected();
            inputs.encoderConnected[j] = module.getEncoder().isConnected();
        }
        inputs.gyroConnected = drivetrain.getPigeon2().isConnected();
    }

    @Override
    public void setRobotVelocity(ChassisVelocities velocity) {
        velocityRequest.Velocity = velocity;
        drivetrain.setControl(velocityRequest);
    }

    @Override
    public void lockWheels() {
        drivetrain.setControl(brakeRequest);
    }

    @Override
    public void idle() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public void close() {
        drivetrain.close();
    }
}
