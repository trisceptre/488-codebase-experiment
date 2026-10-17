package first.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO, AutoCloseable {
    private final Servo left;
    private final Servo right;

    public HoodIOServo(int leftPwmChannel, int rightPwmChannel) {
        left = new Servo(leftPwmChannel);
        right = new Servo(rightPwmChannel);
    }

    @Override
    public void setServoPosition(double position) {
        left.set(position);
        right.set(position);
    }

    @Override
    public void close() {
        left.close();
        right.close();
    }
}
