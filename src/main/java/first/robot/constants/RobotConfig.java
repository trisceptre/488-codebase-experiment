package first.robot.constants;

import edu.wpi.first.apriltag.AprilTagFields;
import first.robot.subsystems.drive.SwerveConfig;
import first.robot.vision.CameraInfo;
import java.util.List;
import java.util.Optional;

public record RobotConfig(
        SwerveConfig drive,
        List<CameraInfo> cameras,
        AprilTagFields field,
        Optional<Candle> lights,
        int pdhCanId,
        Optional<Superstructure> superstructure) {

    public record Candle(int canId, String bus, int ledCount) {}
}
