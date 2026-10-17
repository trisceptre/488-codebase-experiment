package first.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import first.robot.util.motor.MotorIOSim;

public class IntakeDeployIOSim implements IntakeDeployIO {
    private static final double GEARING = 60.0;
    private static final double MOI_KG_M2 = 0.005;
    private static final double DT = MotorIOSim.LOOP_PERIOD_SECONDS;

    private final DCMotorSim sim;
    private final IntakeDeployConfig config;
    private final PIDController feedback;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private Double goalRad = null;
    private double appliedVolts = 0.0;

    public IntakeDeployIOSim(IntakeDeployConfig config, double startRad) {
        this.config = config;
        var gearbox = DCMotor.getKrakenX60(1);
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, MOI_KG_M2, GEARING), gearbox);
        sim.setState(startRad, 0.0);
        feedback = new PIDController(config.kP(), 0.0, 0.0, DT);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(config.cruiseRotationsPerSec()),
                Units.rotationsToRadians(config.accelRotationsPerSecSq())));
        setpoint = new TrapezoidProfile.State(startRad, 0.0);
    }

    public double positionRad() {
        return sim.getAngularPositionRad();
    }

    @Override
    public void updateInputs(IntakeDeployIOInputs inputs) {
        if (goalRad != null) {
            setpoint = profile.calculate(DT, setpoint, new TrapezoidProfile.State(goalRad, 0.0));
            double volts = feedback.calculate(
                    Units.radiansToRotations(sim.getAngularPositionRad()), Units.radiansToRotations(setpoint.position));
            appliedVolts = MathUtil.clamp(volts, -config.peakVolts(), config.peakVolts());
        }
        sim.setInputVoltage(appliedVolts);
        sim.update(DT);
        inputs.motorConnected = true;
        inputs.encoderConnected = true;
        inputs.positionRad = sim.getAngularPositionRad();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.statorAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.supplyAmps = inputs.statorAmps * Math.abs(appliedVolts) / 12.0;
    }

    @Override
    public void setPositionGoal(double positionRad) {
        if (goalRad == null) {
            setpoint = new TrapezoidProfile.State(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec());
            feedback.reset();
        }
        goalRad = positionRad;
    }

    @Override
    public void setVoltage(double volts) {
        goalRad = null;
        appliedVolts = MathUtil.clamp(volts, -config.peakVolts(), config.peakVolts());
    }

    @Override
    public void stop() {
        goalRad = null;
        appliedVolts = 0.0;
    }
}
