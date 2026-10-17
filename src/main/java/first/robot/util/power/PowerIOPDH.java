package first.robot.util.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerIOPDH implements PowerIO {
    private final PowerDistribution pdh;

    public PowerIOPDH(int canId) {
        pdh = new PowerDistribution(canId, ModuleType.kRev);
    }

    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.voltage = pdh.getVoltage();
        inputs.totalCurrentAmps = pdh.getTotalCurrent();
        inputs.connected = inputs.voltage > 0.0;
    }
}
