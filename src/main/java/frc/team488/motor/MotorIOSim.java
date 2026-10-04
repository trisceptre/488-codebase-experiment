package frc.team488.motor;

import frc.team488.motor.MotorConfig.SimModel;
import java.util.Arrays;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Simulated {@link MotorIO} backed by WPILib physics. Closed-loop feedback mirrors the TalonFX: kP
 * and kD act on mechanism rotations, and {@code feedforwardVolts} is added on top.
 */
public class MotorIOSim implements MotorIO {
    public static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double MAX_VOLTS = 12.0;
    private static final double AMBIENT_CELSIUS = 25.0;

    private final MotorConfig config;
    private final Plant plant;
    private final PIDController feedback;
    private Mode lastMode = Mode.NEUTRAL;
    private double appliedVolts = 0.0;
    private boolean connected = true;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        this.plant = Plant.create(config);
        this.feedback =
                new PIDController(config.gains().kP(), 0.0, config.gains().kD(), LOOP_PERIOD_SECONDS);
    }

    /** Simulates the motor dropping off (or rejoining) the CAN bus. */
    public void setDisconnected(boolean disconnected) {
        connected = !disconnected;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        plant.step(appliedVolts, LOOP_PERIOD_SECONDS);
        inputs.connected = connected;
        inputs.positionRad = plant.positionRad();
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
            lastMode = outputs.mode;
        }
        double volts = switch (outputs.mode) {
            case NEUTRAL -> 0.0;
            case VOLTAGE -> outputs.value;
            case VELOCITY ->
                feedback.calculate(rotations(plant.velocityRadPerSec()), rotations(outputs.value))
                        + outputs.feedforwardVolts;
            case POSITION ->
                feedback.calculate(rotations(plant.positionRad()), rotations(outputs.value)) + outputs.feedforwardVolts;
        };
        appliedVolts = Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    }

    private static double rotations(double radians) {
        return Units.radiansToRotations(radians);
    }

    /** A physics model reporting mechanism radians. */
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

    /** Reports the drum angle: carriage meters divided by drum radius. */
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
