package first.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public final class LoggedTracer {
    private static double startTime = -1.0;

    private LoggedTracer() {}

    public static void reset() {
        startTime = Timer.getTimestamp();
    }

    public static void record(String epochName) {
        double now = Timer.getTimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}
