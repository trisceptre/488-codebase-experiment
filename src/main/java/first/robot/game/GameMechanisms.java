package first.robot.game;

import edu.wpi.first.math.util.Units;
import first.robot.LoopParticipant;
import first.robot.constants.Constants;
import first.robot.constants.Superstructure;
import first.robot.subsystems.climber.Climber;
import first.robot.subsystems.climber.ClimberIO;
import first.robot.subsystems.climber.ClimberIODigital;
import first.robot.subsystems.collector.Collector;
import first.robot.subsystems.feeder.Feeder;
import first.robot.subsystems.hood.Hood;
import first.robot.subsystems.hood.HoodIO;
import first.robot.subsystems.hood.HoodIOServo;
import first.robot.subsystems.hopper.Hopper;
import first.robot.subsystems.intake.IntakeDeploy;
import first.robot.subsystems.intake.IntakeDeployIO;
import first.robot.subsystems.intake.IntakeDeployIOSim;
import first.robot.subsystems.intake.IntakeDeployIOTalonFX;
import first.robot.subsystems.shooter.Shooter;
import first.robot.util.motor.MotorConfig;
import first.robot.util.motor.MotorIO;
import first.robot.util.motor.MotorIOSim;
import first.robot.util.motor.MotorIOTalonFX;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public record GameMechanisms(
        Shooter shooter,
        Hood hood,
        Feeder feeder,
        Hopper hopper,
        Collector collector,
        IntakeDeploy intakeDeploy,
        Optional<Climber> climber,
        Optional<IntakeDeployIOSim> intakeDeploySim) {

    private static final double SIM_START_DEG = -10.0;

    public static GameMechanisms create(Superstructure hardware) {
        Constants.Mode mode = Constants.currentMode;
        List<MotorIO> shooterIos =
                hardware.shooter().stream().map(c -> motorIO(c, mode)).toList();

        Optional<IntakeDeployIOSim> intakeSim = mode == Constants.Mode.SIM
                ? Optional.of(new IntakeDeployIOSim(hardware.intakeDeploy(), Units.degreesToRadians(SIM_START_DEG)))
                : Optional.empty();
        IntakeDeployIO intakeIo = switch (mode) {
            case REAL -> new IntakeDeployIOTalonFX(hardware.intakeDeploy());
            case SIM -> intakeSim.get();
            case REPLAY -> new IntakeDeployIO() {};
        };
        HoodIO hoodIo = mode == Constants.Mode.REAL
                ? new HoodIOServo(hardware.hoodLeftPwm(), hardware.hoodRightPwm())
                : new HoodIO() {};

        Optional<Climber> climber = hardware.climber()
                .map(c -> new Climber(
                        c.motors(),
                        motorIO(c.motors(), mode),
                        mode == Constants.Mode.REAL ? new ClimberIODigital(c.sensorDio()) : new ClimberIO() {}));

        return new GameMechanisms(
                new Shooter(hardware.shooter(), shooterIos),
                new Hood(hoodIo),
                new Feeder(hardware.feeder(), motorIO(hardware.feeder(), mode)),
                new Hopper(hardware.hopper(), motorIO(hardware.hopper(), mode)),
                new Collector(hardware.collector(), motorIO(hardware.collector(), mode)),
                new IntakeDeploy(intakeIo),
                climber,
                intakeSim);
    }

    private static MotorIO motorIO(MotorConfig config, Constants.Mode mode) {
        return switch (mode) {
            case REAL -> new MotorIOTalonFX(config);
            case SIM -> new MotorIOSim(config);
            case REPLAY -> new MotorIO() {};
        };
    }

    public List<LoopParticipant> loopParticipants() {
        var participants =
                new ArrayList<LoopParticipant>(List.of(shooter, hood, feeder, hopper, collector, intakeDeploy));
        climber.ifPresent(participants::add);
        return participants;
    }
}
