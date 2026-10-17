package first.robot.subsystems.lights;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import java.util.List;

public class LightsIOCANdle implements LightsIO {
    private final CANdle candle;
    private final int[] firstLed;
    private final int[] lastLed;

    public LightsIOCANdle(int canId, String canBus, List<Integer> segmentLengths, double brightness) {
        candle = new CANdle(canId, new CANBus(canBus));
        var config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = brightness;
        candle.getConfigurator().apply(config);

        firstLed = new int[segmentLengths.size()];
        lastLed = new int[segmentLengths.size()];
        int next = 0;
        for (int i = 0; i < segmentLengths.size(); i++) {
            firstLed[i] = next;
            next += segmentLengths.get(i);
            lastLed[i] = next - 1;
        }
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.connected = candle.isConnected();
    }

    @Override
    public void apply(int segment, LightPattern pattern) {
        int first = firstLed[segment];
        int last = lastLed[segment];
        candle.setControl(new EmptyAnimation(segment));
        if (pattern instanceof LightPattern.Solid solid) {
            candle.setControl(new SolidColor(first, last).withColor(new RGBWColor(solid.color())));
        } else if (pattern instanceof LightPattern.Larson larson) {
            candle.setControl(new LarsonAnimation(first, last)
                    .withSlot(segment)
                    .withColor(new RGBWColor(larson.color()))
                    .withSize(2)
                    .withFrameRate(larson.hz())
                    .withBounceMode(
                            larson.bounce() == LightPattern.Bounce.BACK
                                    ? LarsonBounceValue.Back
                                    : LarsonBounceValue.Front));
        } else if (pattern instanceof LightPattern.Blink blink) {
            candle.setControl(new StrobeAnimation(first, last)
                    .withSlot(segment)
                    .withColor(new RGBWColor(blink.color()))
                    .withFrameRate(blink.hz()));
        } else {
            candle.setControl(new SolidColor(first, last).withColor(new RGBWColor()));
        }
    }
}
