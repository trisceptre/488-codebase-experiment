package first.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public final class PhoenixUtil {
    private static final Map<CANBus, List<BaseStatusSignal>> SIGNALS_BY_BUS = new LinkedHashMap<>();

    private PhoenixUtil() {}

    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        StatusCode status = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            status = command.get();
            if (status.isOK()) {
                break;
            }
        }
        return status;
    }

    public static synchronized void registerSignals(CANBus bus, BaseStatusSignal... signals) {
        SIGNALS_BY_BUS.computeIfAbsent(bus, _ -> new ArrayList<>()).addAll(List.of(signals));
    }

    public static synchronized void refreshAll() {
        for (List<BaseStatusSignal> signals : SIGNALS_BY_BUS.values()) {
            BaseStatusSignal.refreshAll(signals);
        }
    }
}
