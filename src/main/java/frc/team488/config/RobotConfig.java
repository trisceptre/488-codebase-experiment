package frc.team488.config;

import frc.team488.mechanisms.drive.SwerveConfig;
import frc.team488.vision.CameraInfo;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.bus.CANPort;

/** All hardware configuration for one robot. */
public record RobotConfig(
        SwerveConfig drive,
        List<CameraInfo> cameras,
        Fields field,
        int candleId,
        String candleBus,
        int candleLedCount,
        CANPort pdhPort) {}
