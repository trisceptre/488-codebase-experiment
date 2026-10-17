package first.robot.constants;

import first.robot.subsystems.intake.IntakeDeployConfig;
import first.robot.util.motor.MotorConfig;
import java.util.List;
import java.util.Optional;

public record Superstructure(
        List<MotorConfig> shooter,
        int hoodLeftPwm,
        int hoodRightPwm,
        MotorConfig feeder,
        MotorConfig hopper,
        MotorConfig collector,
        IntakeDeployConfig intakeDeploy,
        Optional<Climber> climber) {

    public Superstructure {
        shooter = List.copyOf(shooter);
    }

    public record Climber(MotorConfig motors, int sensorDio) {}
}
