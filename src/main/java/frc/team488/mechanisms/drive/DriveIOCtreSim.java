package frc.team488.mechanisms.drive;

import com.ctre.phoenix6.Utils;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

/** CTRE drivetrain with CTRE's physics sim stepped every 5 ms. */
public class DriveIOCtreSim extends DriveIOCtre {
    private static final double SIM_PERIOD_SECONDS = 0.005;

    private final Notifier simNotifier;
    private double lastSimTime;

    public DriveIOCtreSim(SwerveConfig config) {
        super(config);
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            drivetrain.updateSimState(now - lastSimTime, RobotController.getBatteryVoltage());
            lastSimTime = now;
        });
        simNotifier.startPeriodic(SIM_PERIOD_SECONDS);
    }

    /** CTRE's own odometry pose; in simulation it is ground truth (used to seed vision sim). */
    public Pose2d truePose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void close() {
        simNotifier.close();
        super.close();
    }
}
