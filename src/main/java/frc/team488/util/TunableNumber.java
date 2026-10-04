package frc.team488.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that is a constant on the field and editable from the dashboard while tuning. With
 * {@link #TUNING_MODE} off it publishes nothing and always returns its default.
 */
public final class TunableNumber implements DoubleSupplier {
    /** Turn on to expose every TunableNumber under /Tuning. Keep it off for competition. */
    public static final boolean TUNING_MODE = false;

    private static final String TABLE = "/Tuning/";

    private final double defaultValue;
    private final LoggedNetworkNumber networkNumber;
    private final Map<Integer, Double> lastValues = new HashMap<>();

    public TunableNumber(String key, double defaultValue) {
        this(key, defaultValue, TUNING_MODE);
    }

    TunableNumber(String key, double defaultValue, boolean tuningMode) {
        this.defaultValue = defaultValue;
        this.networkNumber = tuningMode ? new LoggedNetworkNumber(TABLE + key, defaultValue) : null;
    }

    public double get() {
        return networkNumber == null ? defaultValue : networkNumber.get();
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    /**
     * Returns true the first time it's called with {@code id}, and afterwards whenever the value
     * differs from the one seen at the previous call with the same {@code id}.
     */
    public boolean hasChanged(int id) {
        double value = get();
        Double last = lastValues.put(id, value);
        return last == null || last != value;
    }
}
