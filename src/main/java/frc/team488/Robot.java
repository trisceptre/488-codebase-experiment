package frc.team488;

import frc.team488.config.RobotConfig;
import frc.team488.config.RobotConfigs;
import frc.team488.config.RobotIdentity;
import frc.team488.controls.TeleopBindings;
import frc.team488.mechanisms.drive.Drive;
import frc.team488.mechanisms.drive.DriveIO;
import frc.team488.mechanisms.drive.DriveIOCtre;
import frc.team488.mechanisms.drive.DriveIOCtreSim;
import frc.team488.mechanisms.lights.Lights;
import frc.team488.mechanisms.lights.LightsIO;
import frc.team488.mechanisms.lights.LightsIOCandle;
import frc.team488.motor.CtreSignals;
import frc.team488.power.PowerIO;
import frc.team488.power.PowerIOPdh;
import frc.team488.power.PowerIOSim;
import frc.team488.power.VoltageMonitor;
import frc.team488.util.LoggingSetup;
import frc.team488.vision.AprilTagVisionIO;
import frc.team488.vision.AprilTagVisionIOPhoton;
import frc.team488.vision.AprilTagVisionIOSim;
import frc.team488.vision.CameraInfo;
import frc.team488.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.fields.Field;
import org.wpilib.util.Alert;

/**
 * Owns the subsystems and runs the loop in a fixed order: refresh CAN signals, update inputs
 * (drive odometry, then vision, then pose), run the command scheduler, then apply outputs.
 */
public class Robot extends LoggedRobot {
    private final Scheduler scheduler = Scheduler.getDefault();
    private final List<LoopParticipant> loopOrder;
    private final DriveIOCtreSim driveSim;
    private final VisionSystemSim visionSim;

    public Robot() {
        LoggingSetup.Mode mode = LoggingSetup.currentMode();
        LoggingSetup.start(this, mode);
        RobotIdentity identity = RobotIdentity.detect();
        Logger.recordMetadata("RobotIdentity", identity.name());
        RobotConfig config = RobotConfigs.forIdentity(identity);
        Field field = Field.loadField(config.field());

        Pose pose = new Pose(config.drive().kinematics(), config.drive().moduleCount());

        DriveIO driveIO;
        switch (mode) {
            case REAL -> {
                driveIO = new DriveIOCtre(config.drive());
                driveSim = null;
            }
            case SIM -> {
                driveSim = new DriveIOCtreSim(config.drive());
                driveIO = driveSim;
            }
            default -> {
                driveIO = new DriveIO() {};
                driveSim = null;
            }
        }
        Drive drive = new Drive(config.drive(), driveIO, pose);

        visionSim = mode == LoggingSetup.Mode.SIM ? new VisionSystemSim("main") : null;
        if (visionSim != null) {
            visionSim.addAprilTags(field);
        }
        List<AprilTagVisionIO> cameraIOs = new ArrayList<>();
        for (CameraInfo camera : config.cameras()) {
            cameraIOs.add(
                    switch (mode) {
                        case REAL -> new AprilTagVisionIOPhoton(camera, field);
                        case SIM -> new AprilTagVisionIOSim(camera, field, visionSim);
                        case REPLAY -> new AprilTagVisionIO() {};
                    });
        }
        Vision vision = new Vision(config.cameras(), cameraIOs, field, pose);

        Lights lights = new Lights(
                mode == LoggingSetup.Mode.REAL
                        ? new LightsIOCandle(config.candleId(), config.candleBus(), config.candleLedCount())
                        : new LightsIO() {});
        PowerIO powerIO = switch (mode) {
            case REAL -> new PowerIOPdh(config.pdhPort());
            case SIM -> new PowerIOSim();
            case REPLAY -> new PowerIO() {};
        };
        VoltageMonitor voltageMonitor = new VoltageMonitor(powerIO);

        loopOrder = List.of(drive, vision, pose, lights, voltageMonitor);

        boolean offsetsMissing = config.drive().modules().stream().anyMatch(m -> m.EncoderOffset == 0.0);
        new Alert("Drive/OffsetsMissing", "CANcoder offsets not set in DriveConstants", Alert.Level.HIGH)
                .set(mode == LoggingSetup.Mode.REAL && offsetsMissing);

        new TeleopBindings(new CommandGamepad(0), drive, pose);
    }

    @Override
    public void robotPeriodic() {
        CtreSignals.refreshAll();
        for (LoopParticipant participant : loopOrder) {
            participant.updateInputs();
        }
        scheduler.run();
        for (LoopParticipant participant : loopOrder) {
            participant.applyOutputs();
        }
        Logger.recordOutput(
                "Scheduler/Running",
                scheduler.getRunningCommands().stream().map(Command::name).toArray(String[]::new));
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && driveSim != null) {
            visionSim.update(driveSim.truePose());
        }
    }
}
