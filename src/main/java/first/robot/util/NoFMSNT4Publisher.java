package first.robot.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public final class NoFMSNT4Publisher implements LogDataReceiver {
    private static final String FMS_ATTACHED_KEY = "DriverStation/FMSAttached";

    private final LogDataReceiver delegate;

    public NoFMSNT4Publisher() {
        this(new NT4Publisher());
    }

    NoFMSNT4Publisher(LogDataReceiver delegate) {
        this.delegate = delegate;
    }

    @Override
    public void start() {
        delegate.start();
    }

    @Override
    public void end() {
        delegate.end();
    }

    @Override
    public void putTable(LogTable table) throws InterruptedException {
        if (!table.get(FMS_ATTACHED_KEY, false)) {
            delegate.putTable(table);
        }
    }
}
