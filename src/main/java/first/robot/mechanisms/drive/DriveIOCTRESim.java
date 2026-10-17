package first.robot.mechanisms.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class DriveIOCTRESim extends DriveIOCTRE {
    private static final double SIM_PERIOD_SECONDS = 0.005;

    private final Notifier simNotifier;
    private double lastSimTime;

    public DriveIOCTRESim(SwerveConfig config) {
        super(config);
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            drivetrain.updateSimState(now - lastSimTime, RobotController.getBatteryVoltage());
            lastSimTime = now;
        });
        simNotifier.startPeriodic(SIM_PERIOD_SECONDS);
    }

    public Pose2d truePose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void close() {
        simNotifier.close();
        super.close();
    }
}
