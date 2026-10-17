package first.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class LoggedTunableNumber implements DoubleSupplier {
    public enum Level {
        IMPORTANT,
        DEBUG
    }

    private final LoggedNetworkNumber networkNumber;
    private final Map<Integer, Double> lastValues = new HashMap<>();

    public LoggedTunableNumber(String key, double defaultValue) {
        this(key, defaultValue, Level.IMPORTANT);
    }

    public LoggedTunableNumber(String key, double defaultValue, Level level) {
        String path = switch (level) {
            case IMPORTANT -> Preferences.getNetworkTable().getPath() + "/" + key;
            case DEBUG -> "/Tuning/" + key;
        };
        networkNumber = new LoggedNetworkNumber(path, defaultValue);
        if (level == Level.IMPORTANT) {
            NetworkTableInstance.getDefault().getTopic(path).setPersistent(true);
        }
    }

    public double get() {
        return networkNumber.get();
    }

    public void set(double value) {
        networkNumber.set(value);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    public boolean hasChanged(int id) {
        double value = get();
        Double last = lastValues.put(id, value);
        return last == null || last != value;
    }
}
