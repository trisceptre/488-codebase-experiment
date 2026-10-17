package first.robot.constants;

import first.robot.mechanisms.drive.SwerveConfig;
import first.robot.vision.CameraInfo;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.bus.CANPort;

public record RobotConfig(
        SwerveConfig drive,
        List<CameraInfo> cameras,
        Fields field,
        int candleId,
        String candleBus,
        int candleLedCount,
        CANPort pdhPort) {}
