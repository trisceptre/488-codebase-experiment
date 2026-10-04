package frc.team488;

import org.wpilib.framework.RobotBase;

/** Entry point. Do not add initialization here; it belongs in {@link Robot}. */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
