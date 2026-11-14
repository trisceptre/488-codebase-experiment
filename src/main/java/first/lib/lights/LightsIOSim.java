package first.lib.lights;

public class LightsIOSim implements LightsIO {
    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.connected = true;
    }
}
