package first.robot.mechanisms.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

final class FakeDriveIO implements DriveIO {
    enum Request {
        NONE,
        VELOCITY,
        X,
        IDLE
    }

    private static final double DT = 0.02;
    private final SwerveDriveKinematics kinematics;
    private final int modules;
    private final double[] distances;
    private final double[] angles;
    private double yaw = 0.0;
    Request last = Request.NONE;
    ChassisSpeeds robotVelocity = new ChassisSpeeds();
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
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotVelocity);
            for (int j = 0; j < modules; j++) {
                distances[j] += states[j].speedMetersPerSecond * DT;
                angles[j] = states[j].angle.getRadians();
            }
            yaw += robotVelocity.omegaRadiansPerSecond * DT;
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
    public void setRobotVelocity(ChassisSpeeds velocity) {
        last = Request.VELOCITY;
        robotVelocity = velocity;
    }

    @Override
    public void stopWithX() {
        last = Request.X;
    }

    @Override
    public void idle() {
        last = Request.IDLE;
    }
}
