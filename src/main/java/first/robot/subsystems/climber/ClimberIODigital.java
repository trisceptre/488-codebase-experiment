package first.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIODigital implements ClimberIO, AutoCloseable {
    private final DigitalInput sensor;

    public ClimberIODigital(int dioChannel) {
        sensor = new DigitalInput(dioChannel);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.atRetractedLimit = sensor.get();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
