package frc.team488.power;

import org.wpilib.system.RobotController;

/** Simulated battery voltage from the WPILib sim. */
public class PowerIOSim implements PowerIO {
    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.connected = true;
        inputs.voltage = RobotController.getBatteryVoltage();
        inputs.totalCurrentAmps = 0.0;
    }
}
