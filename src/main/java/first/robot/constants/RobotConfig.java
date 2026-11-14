package first.robot.constants;

import edu.wpi.first.apriltag.AprilTagFields;
import first.lib.drive.SwerveConfig;
import first.lib.vision.CameraInfo;
import java.util.List;
import java.util.Optional;

public record RobotConfig(
        SwerveConfig drive,
        List<CameraInfo> cameras,
        AprilTagFields field,
        Optional<Candle> lights,
        int pdhCanId,
        Optional<Superstructure> superstructure) {

    public record Candle(int canId, String bus, List<Integer> segments, double brightness) {
        public Candle {
            segments = List.copyOf(segments);
        }

        public int ledCount() {
            return segments.stream().mapToInt(Integer::intValue).sum();
        }
    }
}
