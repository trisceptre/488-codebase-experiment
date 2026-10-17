package first.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import first.robot.opmodes.teleop.TeleopBindings;
import first.robot.util.LoggedTracer;
import first.robot.util.NoFMSNT4Publisher;
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
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.simulation.VisionSystemSim;

public class Robot extends LoggedRobot {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final Set<Command> runningCommands = new LinkedHashSet<>();
    private final List<LoopParticipant> loopOrder;
    private final DriveIOCTRESim driveSim;
    private final VisionSystemSim visionSim;

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
                Logger.addDataReceiver(new NoFMSNT4Publisher());
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new NT4Publisher());
                DriverStation.silenceJoystickConnectionWarning(true);
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
        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(config.field());

        RobotState robotState =
                new RobotState(config.drive().kinematics(), config.drive().moduleCount());

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
        Drive drive = new Drive(config.drive(), driveIO, robotState);

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

        Lights lights = new Lights(config.lights()
                .filter(candle -> Constants.currentMode == Constants.Mode.REAL)
                .<LightsIO>map(candle -> new LightsIOCANdle(candle.canId(), candle.bus(), candle.ledCount()))
                .orElseGet(() -> new LightsIO() {}));
        PowerIO powerIO = switch (Constants.currentMode) {
            case REAL -> new PowerIOPDH(config.pdhCanId());
            case SIM -> new PowerIOSim();
            case REPLAY -> new PowerIO() {};
        };
        VoltageMonitor voltageMonitor = new VoltageMonitor(powerIO);

        loopOrder = List.of(drive, vision, robotState, lights, voltageMonitor);

        boolean offsetsMissing = config.drive().modules().stream().anyMatch(m -> m.EncoderOffset == 0.0);
        new Alert("CANcoder offsets not set in TunerConstants", Alert.AlertType.kError)
                .set(Constants.currentMode == Constants.Mode.REAL && offsetsMissing);

        new TeleopBindings(
                new CommandXboxController(DriverConstants.DRIVER_CONTROLLER_PORT),
                new CommandXboxController(DriverConstants.DEBUG_CONTROLLER_PORT),
                drive,
                robotState);

        scheduler.onCommandInitialize(runningCommands::add);
        scheduler.onCommandFinish(runningCommands::remove);
        scheduler.onCommandInterrupt(runningCommands::remove);
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
                runningCommands.stream().map(Command::getName).toArray(String[]::new));
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && driveSim != null) {
            visionSim.update(driveSim.truePose());
        }
    }
}
