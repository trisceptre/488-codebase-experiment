package frc.team488.power;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;

/** REV Power Distribution Hub. */
public class PowerIOPdh implements PowerIO {
    private final PowerDistribution pdh;

    public PowerIOPdh(CANPort port) {
        pdh = new PowerDistribution(port);
    }

    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.voltage = pdh.getVoltage();
        inputs.totalCurrentAmps = pdh.getTotalCurrent();
        inputs.connected = inputs.voltage > 0.0;
    }
}
