package first.robot.util;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;

public final class LoggedTunableBoolean implements BooleanSupplier {
    private final LoggedNetworkBoolean networkBoolean;

    public LoggedTunableBoolean(String key, boolean defaultValue) {
        String path = Preferences.getNetworkTable().getPath() + "/" + key;
        networkBoolean = new LoggedNetworkBoolean(path, defaultValue);
        NetworkTableInstance.getDefault().getTopic(path).setPersistent(true);
    }

    public boolean get() {
        return networkBoolean.get();
    }

    @Override
    public boolean getAsBoolean() {
        return get();
    }
}
