package first.robot.util.power;

import first.robot.LoopParticipant;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.util.Alert;

public final class VoltageMonitor implements LoopParticipant, AutoCloseable {
    private final PowerIO io;
    private final PowerIOInputsAutoLogged inputs = new PowerIOInputsAutoLogged();
    private final LoggedTunableNumber lowVoltage = new LoggedTunableNumber("Power/LowVoltage", 8.0);
    private final Debouncer lowDebouncer = new Debouncer(1.0, Debouncer.DebounceType.RISING);
    private final Alert lowAlert =
            new Alert("Power/LowVoltage", "Battery voltage low: brownout risk", Alert.Level.HIGH);
    private boolean brownoutRisk = false;

    public VoltageMonitor(PowerIO io) {
        this.io = io;
    }

    public boolean isBrownoutRisk() {
        return brownoutRisk;
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Power", inputs);
        brownoutRisk = lowDebouncer.calculate(inputs.connected && inputs.voltage < lowVoltage.get());
        lowAlert.set(brownoutRisk);
    }

    @Override
    public void applyOutputs() {}

    @Override
    public void close() {
        lowAlert.close();
    }
}
