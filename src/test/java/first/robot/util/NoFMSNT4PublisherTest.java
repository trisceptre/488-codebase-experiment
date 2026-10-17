package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

class NoFMSNT4PublisherTest {
    private static final class Recorder implements LogDataReceiver {
        final List<LogTable> tables = new ArrayList<>();

        @Override
        public void putTable(LogTable table) {
            tables.add(table);
        }
    }

    private static LogTable table(boolean fmsAttached) {
        LogTable table = new LogTable(0);
        table.put("DriverStation/FMSAttached", fmsAttached);
        return table;
    }

    @Test
    void forwardsOnlyWhileFmsIsDetached() throws InterruptedException {
        var recorder = new Recorder();
        var gated = new NoFMSNT4Publisher(recorder);
        gated.putTable(table(false));
        gated.putTable(table(true));
        gated.putTable(new LogTable(0));
        assertEquals(2, recorder.tables.size());
    }
}
