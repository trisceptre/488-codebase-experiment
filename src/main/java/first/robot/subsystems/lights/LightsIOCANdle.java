package first.robot.subsystems.lights;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

public class LightsIOCANdle implements LightsIO {
    private final CANdle candle;
    private final int lastLed;

    public LightsIOCANdle(int canId, String canBus, int ledCount) {
        candle = new CANdle(canId, new CANBus(canBus));
        lastLed = ledCount - 1;
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.connected = candle.isConnected();
    }

    @Override
    public void apply(LightPattern pattern) {
        candle.setControl(new EmptyAnimation(0));
        if (pattern instanceof LightPattern.Solid solid) {
            candle.setControl(new SolidColor(0, lastLed).withColor(new RGBWColor(solid.color())));
        } else if (pattern instanceof LightPattern.Larson larson) {
            candle.setControl(new LarsonAnimation(0, lastLed)
                    .withSlot(0)
                    .withColor(new RGBWColor(larson.color()))
                    .withSize(2)
                    .withBounceMode(LarsonBounceValue.Front));
        } else if (pattern instanceof LightPattern.Blink blink) {
            candle.setControl(new StrobeAnimation(0, lastLed).withSlot(0).withColor(new RGBWColor(blink.color())));
        } else {
            candle.setControl(new SolidColor(0, lastLed).withColor(new RGBWColor()));
        }
    }
}
