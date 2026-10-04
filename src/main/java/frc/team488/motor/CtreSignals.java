package frc.team488.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Batches Phoenix status-signal refreshes: one {@link BaseStatusSignal#refreshAll} per CAN bus per
 * loop, instead of one per device. {@code Robot} calls {@link #refreshAll()} first each loop.
 */
public final class CtreSignals {
    private static final Map<CANBus, List<BaseStatusSignal>> SIGNALS_BY_BUS = new LinkedHashMap<>();

    private CtreSignals() {}

    public static synchronized void register(CANBus bus, BaseStatusSignal... signals) {
        SIGNALS_BY_BUS.computeIfAbsent(bus, _ -> new ArrayList<>()).addAll(List.of(signals));
    }

    public static synchronized void refreshAll() {
        for (List<BaseStatusSignal> signals : SIGNALS_BY_BUS.values()) {
            BaseStatusSignal.refreshAll(signals);
        }
    }
}
