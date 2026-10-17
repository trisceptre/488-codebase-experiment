package first.robot.util.power;

import edu.wpi.first.wpilibj.RobotController;

public class PowerIOSim implements PowerIO {
    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.connected = true;
        inputs.voltage = RobotController.getBatteryVoltage();
        inputs.totalCurrentAmps = 0.0;
    }
}
