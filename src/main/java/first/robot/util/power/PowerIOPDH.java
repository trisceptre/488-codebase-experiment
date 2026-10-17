package first.robot.util.power;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;

public class PowerIOPDH implements PowerIO {
    private final PowerDistribution pdh;

    public PowerIOPDH(CANPort port) {
        pdh = new PowerDistribution(port);
    }

    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.voltage = pdh.getVoltage();
        inputs.totalCurrentAmps = pdh.getTotalCurrent();
        inputs.connected = inputs.voltage > 0.0;
    }
}
