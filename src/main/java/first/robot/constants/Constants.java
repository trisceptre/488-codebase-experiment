package first.robot.constants;

import org.wpilib.framework.RobotBase;

public final class Constants {
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : "true".equals(System.getenv("ROBOT_REPLAY")) ? Mode.REPLAY : Mode.SIM;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    private Constants() {}
}
