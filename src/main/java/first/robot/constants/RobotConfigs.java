package first.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import first.robot.generated.TunerConstants;
import first.robot.generated.TunerConstants2025;
import first.robot.subsystems.drive.SwerveConfig;
import first.robot.vision.CameraInfo;
import java.util.List;
import java.util.Optional;

public final class RobotConfigs {
    private RobotConfigs() {}

    private static final int PDH_CAN_ID = 1;

    private static Transform3d mount(double x, double y, double z, double pitchDeg, double yawDeg) {
        return new Transform3d(
                new Translation3d(x, y, z), new Rotation3d(0.0, Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
    }

    private static double inches(double value) {
        return Inches.of(value).in(Meters);
    }

    private static final List<CameraInfo> CAMERAS_2026 = List.of(
            new CameraInfo("Left", "Apriltag_Left_Camera", mount(-0.28, 0.2965, 0.19, -25.5, 90), 1.0),
            new CameraInfo("Right", "Apriltag_Right_Camera", mount(-0.28, -0.2965, 0.19, -25.5, 270), 1.0),
            new CameraInfo("Front", "Apriltag_Front_Camera", mount(0.0, inches(0.25), inches(20.075958), -23, 0), 1.0),
            new CameraInfo("Back", "Apriltag_Back_Camera", mount(-0.3429, 0.0, 0.487, -15, 180), 1.0));

    private static final List<CameraInfo> CAMERAS_2025 = List.of(
            new CameraInfo(
                    "FrontLeft",
                    "Apriltag_FrontLeft_Camera",
                    mount(inches(10.14), inches(6.535), inches(6.7), -21, 0),
                    1.0),
            new CameraInfo(
                    "FrontRight",
                    "Apriltag_FrontRight_Camera",
                    mount(inches(10.14), inches(-6.535), inches(6.7), -21, 0),
                    1.0));

    private static final SwerveConfig DRIVE_2026 = SwerveConfig.create(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private static final SwerveConfig DRIVE_2025 = SwerveConfig.create(
            TunerConstants2025.DrivetrainConstants,
            TunerConstants2025.FrontLeft,
            TunerConstants2025.FrontRight,
            TunerConstants2025.BackLeft,
            TunerConstants2025.BackRight,
            TunerConstants2025.kSpeedAt12Volts.in(MetersPerSecond));

    public static final RobotConfig COMP_2026 = new RobotConfig(
            DRIVE_2026,
            CAMERAS_2026,
            AprilTagFields.k2026RebuiltWelded,
            Optional.of(new RobotConfig.Candle(57, "*", List.of(8, 30, 30), 0.15)),
            PDH_CAN_ID,
            Optional.of(Hardware2026.SUPERSTRUCTURE));

    public static final RobotConfig CHASSIS_2025 = new RobotConfig(
            DRIVE_2025,
            CAMERAS_2025,
            AprilTagFields.k2026RebuiltWelded,
            Optional.empty(),
            PDH_CAN_ID,
            Optional.empty());

    public static final RobotConfig SIM = COMP_2026;

    public static RobotConfig forIdentity(RobotIdentity identity) {
        return switch (identity) {
            case COMP_2026 -> COMP_2026;
            case CHASSIS_2025 -> CHASSIS_2025;
            case SIM -> SIM;
        };
    }
}
