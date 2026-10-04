package frc.team488.mechanisms.drive;

import java.util.Arrays;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.Timer;

/** Deterministic drivetrain: integrates the last command into one odometry sample per loop. */
final class FakeDriveIO implements DriveIO {
    enum Request {
        NONE,
        VELOCITY,
        LOCK,
        IDLE
    }

    private static final double DT = 0.02;
    private final SwerveDriveKinematics kinematics;
    private final int modules;
    private final double[] distances;
    private final double[] angles;
    private double yaw = 0.0;
    Request last = Request.NONE;
    ChassisVelocities robotVelocity = new ChassisVelocities();
    boolean gyroConnected = true;

    FakeDriveIO(SwerveConfig config) {
        kinematics = config.kinematics();
        modules = config.moduleCount();
        distances = new double[modules];
        angles = new double[modules];
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        if (last == Request.VELOCITY) {
            SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(robotVelocity);
            for (int j = 0; j < modules; j++) {
                distances[j] += states[j].velocity * DT;
                angles[j] = states[j].angle.getRadians();
            }
            yaw += robotVelocity.omega * DT;
        }
        inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
        inputs.odometryYawRad = new double[] {yaw};
        inputs.odometryModuleDistancesMeters = distances.clone();
        inputs.odometryModuleAnglesRad = angles.clone();
        inputs.driveConnected = allTrue();
        inputs.steerConnected = allTrue();
        inputs.encoderConnected = allTrue();
        inputs.gyroConnected = gyroConnected;
    }

    private boolean[] allTrue() {
        var a = new boolean[modules];
        Arrays.fill(a, true);
        return a;
    }

    @Override
    public void setRobotVelocity(ChassisVelocities velocity) {
        last = Request.VELOCITY;
        robotVelocity = velocity;
    }

    @Override
    public void lockWheels() {
        last = Request.LOCK;
    }

    @Override
    public void idle() {
        last = Request.IDLE;
    }
}
