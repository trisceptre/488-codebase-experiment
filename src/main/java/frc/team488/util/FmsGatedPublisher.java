package frc.team488.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/**
 * Forwards log tables to another receiver (normally {@code NT4Publisher}) only while no FMS is
 * attached. On the field this keeps thousands of values off NetworkTables; everything is still
 * written to the log file.
 */
public final class FmsGatedPublisher implements LogDataReceiver {
    private static final String FMS_ATTACHED_KEY = "DriverStation/FMSAttached";

    private final LogDataReceiver delegate;

    public FmsGatedPublisher(LogDataReceiver delegate) {
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
