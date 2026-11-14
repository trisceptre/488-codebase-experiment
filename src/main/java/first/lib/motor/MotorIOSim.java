package first.lib.motor;

import first.lib.motor.MotorConfig.SimModel;
import java.util.Arrays;
import java.util.Optional;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

public class MotorIOSim implements MotorIO {
    public static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double MAX_VOLTS = 12.0;
    private static final double AMBIENT_CELSIUS = 25.0;

    private final MotorConfig config;
    private final Plant plant;
    private final PIDController feedback;
    private final Optional<TrapezoidProfile> profile;
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();
    private Mode lastMode = Mode.NEUTRAL;
    private double appliedVolts = 0.0;
    private double sensorOffsetRad = 0.0;
    private boolean connected = true;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        this.plant = Plant.create(config);
        this.feedback =
                new PIDController(config.gains().kP(), 0.0, config.gains().kD(), LOOP_PERIOD_SECONDS);
        this.profile = config.shaping()
                .motionMagic()
                .map(mm -> new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(mm.cruiseRadPerSec(), mm.accelRadPerSecSq())));
    }

    public void setDisconnected(boolean disconnected) {
        connected = !disconnected;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        plant.step(appliedVolts, LOOP_PERIOD_SECONDS);
        inputs.connected = connected;
        inputs.positionRad = sensorPositionRad();
        inputs.velocityRadPerSec = plant.velocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.statorAmps = Math.abs(plant.currentAmps());
        inputs.supplyAmps = inputs.statorAmps * Math.abs(appliedVolts) / MAX_VOLTS;
        inputs.tempCelsius = AMBIENT_CELSIUS;

        int followerCount = config.followers().size();
        inputs.followerConnected = new boolean[followerCount];
        Arrays.fill(inputs.followerConnected, connected);
        inputs.followerSupplyAmps = new double[followerCount];
        Arrays.fill(inputs.followerSupplyAmps, inputs.supplyAmps);
        inputs.followerTempCelsius = new double[followerCount];
        Arrays.fill(inputs.followerTempCelsius, AMBIENT_CELSIUS);
    }

    @Override
    public void applyOutputs(Outputs outputs) {
        if (outputs.mode != lastMode) {
            feedback.reset();
            profileSetpoint = new TrapezoidProfile.State(sensorPositionRad(), plant.velocityRadPerSec());
            lastMode = outputs.mode;
        }
        double volts = switch (outputs.mode) {
            case NEUTRAL -> 0.0;
            case VOLTAGE -> outputs.value;
            case VELOCITY ->
                feedback.calculate(rotations(plant.velocityRadPerSec()), rotations(outputs.value))
                        + velocityFeedforward(outputs.value, 0.0)
                        + gravityFeedforward();
            case POSITION -> positionVolts(outputs.value);
        };
        if (outputs.mode == Mode.NEUTRAL) {
            appliedVolts = 0.0;
            return;
        }
        volts = ramped(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
        appliedVolts = outputs.ignoreSoftLimits ? volts : softLimited(volts);
    }

    @Override
    public boolean setGains(double kP, double kD) {
        if (!connected) {
            return false;
        }
        feedback.setPID(kP, 0.0, kD);
        return true;
    }

    @Override
    public boolean setPosition(double positionRad) {
        if (!connected) {
            return false;
        }
        sensorOffsetRad = positionRad - plant.positionRad();
        profileSetpoint = new TrapezoidProfile.State(positionRad, plant.velocityRadPerSec());
        return true;
    }

    private double sensorPositionRad() {
        return plant.positionRad() + sensorOffsetRad;
    }

    private double softLimited(double volts) {
        if (config.softLimits().isEmpty()) {
            return volts;
        }
        MotorConfig.SoftLimits limits = config.softLimits().get();
        double position = sensorPositionRad();
        if ((volts > 0.0 && position >= limits.maxRad()) || (volts < 0.0 && position <= limits.minRad())) {
            return 0.0;
        }
        return volts;
    }

    private double ramped(double volts) {
        double rampSeconds = config.shaping().rampSeconds();
        if (rampSeconds <= 0.0) {
            return volts;
        }
        double maxStep = MAX_VOLTS / rampSeconds * LOOP_PERIOD_SECONDS;
        return appliedVolts + Math.clamp(volts - appliedVolts, -maxStep, maxStep);
    }

    private double positionVolts(double goalRad) {
        double setpointRad = goalRad;
        double feedforward = gravityFeedforward();
        if (profile.isPresent()) {
            var next = profile.get()
                    .calculate(LOOP_PERIOD_SECONDS, profileSetpoint, new TrapezoidProfile.State(goalRad, 0.0));
            double accel = (next.velocity - profileSetpoint.velocity) / LOOP_PERIOD_SECONDS;
            feedforward += velocityFeedforward(next.velocity, accel);
            profileSetpoint = next;
            setpointRad = next.position;
        }
        return feedback.calculate(rotations(sensorPositionRad()), rotations(setpointRad)) + feedforward;
    }

    private double velocityFeedforward(double velocityRadPerSec, double accelRadPerSecSq) {
        MotorConfig.Gains gains = config.gains();
        return gains.kS() * Math.signum(velocityRadPerSec)
                + gains.kV() * rotations(velocityRadPerSec)
                + gains.kA() * rotations(accelRadPerSecSq);
    }

    private double gravityFeedforward() {
        MotorConfig.Gains gains = config.gains();
        return switch (gains.gravityType()) {
            case NONE -> 0.0;
            case ELEVATOR -> gains.kG();
            case ARM -> gains.kG() * Math.cos(sensorPositionRad());
        };
    }

    private static double rotations(double radians) {
        return Units.radiansToRotations(radians);
    }

    private interface Plant {
        void step(double volts, double dtSeconds);

        double positionRad();

        double velocityRadPerSec();

        double currentAmps();

        static Plant create(MotorConfig config) {
            double gearing = config.sensorToMechanismRatio();
            return switch (config.simModel()) {
                case SimModel.Flywheel f ->
                    new FlywheelPlant(new FlywheelSim(
                            Models.flywheelFromPhysicalConstants(f.motor(), f.moiKgM2(), gearing), f.motor()));
                case SimModel.Arm a ->
                    new ArmPlant(new SingleJointedArmSim(
                            a.motor(),
                            gearing,
                            a.moiKgM2(),
                            a.lengthMeters(),
                            a.minRad(),
                            a.maxRad(),
                            true,
                            a.minRad()));
                case SimModel.Elevator e ->
                    new ElevatorPlant(
                            new ElevatorSim(
                                    e.motor(),
                                    gearing,
                                    e.carriageMassKg(),
                                    e.drumRadiusMeters(),
                                    e.minMeters(),
                                    e.maxMeters(),
                                    true,
                                    e.minMeters()),
                            e.drumRadiusMeters());
            };
        }
    }

    private static final class FlywheelPlant implements Plant {
        private final FlywheelSim sim;
        private double positionRad = 0.0;

        FlywheelPlant(FlywheelSim sim) {
            this.sim = sim;
        }

        @Override
        public void step(double volts, double dtSeconds) {
            sim.setInputVoltage(volts);
            sim.update(dtSeconds);
            positionRad += sim.getAngularVelocity() * dtSeconds;
        }

        @Override
        public double positionRad() {
            return positionRad;
        }

        @Override
        public double velocityRadPerSec() {
            return sim.getAngularVelocity();
        }

        @Override
        public double currentAmps() {
            return sim.getCurrentDraw();
        }
    }

    private static final class ArmPlant implements Plant {
        private final SingleJointedArmSim sim;

        ArmPlant(SingleJointedArmSim sim) {
            this.sim = sim;
        }

        @Override
        public void step(double volts, double dtSeconds) {
            sim.setInputVoltage(volts);
            sim.update(dtSeconds);
        }

        @Override
        public double positionRad() {
            return sim.getAngle();
        }

        @Override
        public double velocityRadPerSec() {
            return sim.getVelocity();
        }

        @Override
        public double currentAmps() {
            return sim.getCurrentDraw();
        }
    }

    private static final class ElevatorPlant implements Plant {
        private final ElevatorSim sim;
        private final double drumRadiusMeters;

        ElevatorPlant(ElevatorSim sim, double drumRadiusMeters) {
            this.sim = sim;
            this.drumRadiusMeters = drumRadiusMeters;
        }

        @Override
        public void step(double volts, double dtSeconds) {
            sim.setInputVoltage(volts);
            sim.update(dtSeconds);
        }

        @Override
        public double positionRad() {
            return sim.getPosition() / drumRadiusMeters;
        }

        @Override
        public double velocityRadPerSec() {
            return sim.getVelocity() / drumRadiusMeters;
        }

        @Override
        public double currentAmps() {
            return sim.getCurrentDraw();
        }
    }
}
