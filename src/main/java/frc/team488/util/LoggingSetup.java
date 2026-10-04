package frc.team488.util;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.framework.RobotBase;

/** Configures AdvantageKit for the real robot, simulation, or log replay. */
public final class LoggingSetup {
    /** Set by {@code ./gradlew run -Preplay}. */
    private static final String REPLAY_ENV = "ROBOT_REPLAY";

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    private LoggingSetup() {}

    public static Mode currentMode() {
        if (RobotBase.isReal()) {
            return Mode.REAL;
        }
        return "true".equals(System.getenv(REPLAY_ENV)) ? Mode.REPLAY : Mode.SIM;
    }

    /** Adds the receivers for {@code mode} and starts the logger. Call once, first, in the robot constructor. */
    public static void start(LoggedRobot robot, Mode mode) {
        Logger.recordMetadata("Mode", mode.name());
        switch (mode) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new FmsGatedPublisher(new NT4Publisher()));
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                robot.setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
            }
        }
        Logger.start();
    }
}
