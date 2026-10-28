package first.robot;

import first.robot.constants.Constants;
import first.robot.constants.DriverConstants;
import first.robot.constants.RobotConfig;
import first.robot.constants.RobotConfigs;
import first.robot.constants.RobotIdentity;
import first.robot.mechanisms.drive.Drive;
import first.robot.mechanisms.drive.DriveIO;
import first.robot.mechanisms.drive.DriveIOCTRE;
import first.robot.mechanisms.drive.DriveIOCTRESim;
import first.robot.mechanisms.lights.Lights;
import first.robot.mechanisms.lights.LightsIO;
import first.robot.mechanisms.lights.LightsIOCANdle;
import first.robot.util.LoggedTracer;
import first.robot.util.PhoenixUtil;
import first.robot.util.power.PowerIO;
import first.robot.util.power.PowerIOPDH;
import first.robot.util.power.PowerIOSim;
import first.robot.util.power.VoltageMonitor;
import first.robot.vision.CameraInfo;
import first.robot.vision.Vision;
import first.robot.vision.VisionIO;
import first.robot.vision.VisionIOPhotonVision;
import first.robot.vision.VisionIOPhotonVisionSim;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.fields.Field;
import org.wpilib.util.Alert;

public class Robot extends LoggedOpModeRobot {
    public final Drive drive;
    public final RobotState robotState;
    public final CommandGamepad driverController = new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);
    public final CommandGamepad debugController = new CommandGamepad(DriverConstants.DEBUG_CONTROLLER_PORT);

    private final Scheduler scheduler = Scheduler.getDefault();
    private final List<LoopParticipant> loopOrder;
    private final DriveIOCTRESim driveSim;
    private final VisionSystemSim visionSim;
    private final Alert offsetsMissingAlert =
            new Alert("Drive/OffsetsMissing", "CANcoder offsets not set in TunerConstants", Alert.Level.HIGH);

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
        Logger.recordMetadata("Mode", Constants.currentMode.name());

        switch (Constants.currentMode) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter());
                if (!org.wpilib.driverstation.RobotState.isFMSAttached()) {
                    Logger.addDataReceiver(new NT4Publisher());
                }
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
            }
        }
        Logger.start();

        RobotIdentity identity = RobotIdentity.detect();
        Logger.recordMetadata("RobotIdentity", identity.name());
        RobotConfig config = RobotConfigs.forIdentity(identity);
        Field field = Field.loadField(config.field());

        robotState = new RobotState(config.drive().kinematics(), config.drive().moduleCount());

        DriveIO driveIO;
        switch (Constants.currentMode) {
            case REAL -> {
                driveIO = new DriveIOCTRE(config.drive());
                driveSim = null;
            }
            case SIM -> {
                driveSim = new DriveIOCTRESim(config.drive());
                driveIO = driveSim;
            }
            default -> {
                driveIO = new DriveIO() {};
                driveSim = null;
            }
        }
        drive = new Drive(config.drive(), driveIO, robotState);

        visionSim = Constants.currentMode == Constants.Mode.SIM ? new VisionSystemSim("main") : null;
        if (visionSim != null) {
            visionSim.addAprilTags(field);
        }
        List<VisionIO> cameraIOs = new ArrayList<>();
        for (CameraInfo camera : config.cameras()) {
            cameraIOs.add(
                    switch (Constants.currentMode) {
                        case REAL -> new VisionIOPhotonVision(camera, field);
                        case SIM -> new VisionIOPhotonVisionSim(camera, field, visionSim);
                        case REPLAY -> new VisionIO() {};
                    });
        }
        Vision vision = new Vision(config.cameras(), cameraIOs, field, robotState);

        Lights lights = new Lights(
                Constants.currentMode == Constants.Mode.REAL
                        ? new LightsIOCANdle(config.candleId(), config.candleBus(), config.candleLedCount())
                        : new LightsIO() {});
        PowerIO powerIO = switch (Constants.currentMode) {
            case REAL -> new PowerIOPDH(config.pdhPort());
            case SIM -> new PowerIOSim();
            case REPLAY -> new PowerIO() {};
        };
        VoltageMonitor voltageMonitor = new VoltageMonitor(powerIO);

        loopOrder = List.of(drive, vision, robotState, lights, voltageMonitor);

        boolean offsetsMissing = config.drive().modules().stream().anyMatch(m -> m.EncoderOffset == 0.0);
        offsetsMissingAlert.set(Constants.currentMode == Constants.Mode.REAL && offsetsMissing);
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();
        PhoenixUtil.refreshAll();
        LoggedTracer.record("PhoenixRefresh");
        for (LoopParticipant participant : loopOrder) {
            participant.updateInputs();
        }
        LoggedTracer.record("UpdateInputs");
        scheduler.run();
        LoggedTracer.record("Scheduler");
        for (LoopParticipant participant : loopOrder) {
            participant.applyOutputs();
        }
        LoggedTracer.record("ApplyOutputs");
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
