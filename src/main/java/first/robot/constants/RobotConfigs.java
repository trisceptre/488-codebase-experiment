package first.robot.constants;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;

import first.robot.generated.TunerConstants;
import first.robot.mechanisms.drive.SwerveConfig;
import first.robot.vision.CameraInfo;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public final class RobotConfigs {
    private RobotConfigs() {}

    private static Transform3d mount(double x, double y, double z, double pitchDeg, double yawDeg) {
        return new Transform3d(
                new Translation3d(x, y, z), new Rotation3d(0.0, Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
    }

    private static final List<CameraInfo> CAMERAS = List.of(
            new CameraInfo("Left", "AprilTagLeft", mount(-0.28, 0.2965, 0.19, -25.5, 90), 1.0),
            new CameraInfo("Right", "AprilTagRight", mount(-0.28, -0.2965, 0.19, -25.5, 270), 1.0),
            new CameraInfo(
                    "Front",
                    "AprilTagFront",
                    mount(
                            0.0,
                            Inches.of(0.25).in(org.wpilib.units.Units.Meters),
                            Inches.of(20.075958).in(org.wpilib.units.Units.Meters),
                            -23,
                            0),
                    1.0),
            new CameraInfo("Back", "AprilTagBack", mount(-0.3429, 0.0, 0.487, -15, 180), 1.0));

    private static final SwerveConfig DRIVE = SwerveConfig.create(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public static final RobotConfig COMP =
            new RobotConfig(DRIVE, CAMERAS, Fields.FRC_2026_REBUILT_WELDED, 57, "*", 8, CANPort.CAN_S0);

    public static final RobotConfig SIM = COMP;

    public static RobotConfig forIdentity(RobotIdentity identity) {
        return switch (identity) {
            case COMP -> COMP;
            case SIM -> SIM;
        };
    }
}
