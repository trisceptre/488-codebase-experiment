package first.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import first.robot.util.PhoenixUtil;
import first.robot.util.motor.MotorIOTalonFX;

public class IntakeDeployIOTalonFX implements IntakeDeployIO, AutoCloseable {
    private static final int CONFIG_ATTEMPTS = 3;
    private static final double CONFIG_TIMEOUT_SECONDS = 0.25;

    private final TalonFX motor;
    private final CANcoder encoder;
    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Angle> motorPosition;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final Alert configFailedAlert = new Alert("IntakeDeploy motor config failed", Alert.AlertType.kError);

    public IntakeDeployIOTalonFX(IntakeDeployConfig config) {
        var motorBus = MotorIOTalonFX.canBus(config.motor());
        var encoderBus = MotorIOTalonFX.canBus(config.encoder());
        motor = new TalonFX(config.motor().id(), motorBus);
        encoder = new CANcoder(config.encoder().id(), encoderBus);

        var talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.Inverted =
                config.inverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonConfig.Feedback.FeedbackRemoteSensorID = config.encoder().id();
        talonConfig.Slot0.kP = config.kP();
        talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.cruiseRotationsPerSec();
        talonConfig.MotionMagic.MotionMagicAcceleration = config.accelRotationsPerSecSq();
        talonConfig.Voltage.PeakForwardVoltage = config.peakVolts();
        talonConfig.Voltage.PeakReverseVoltage = -config.peakVolts();
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampSeconds();
        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampSeconds();
        talonConfig.CurrentLimits.StatorCurrentLimit = config.statorAmps();
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyAmps();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = config.supplyLowerAmps();
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = config.supplyLowerSeconds();
        var status = PhoenixUtil.tryUntilOk(
                CONFIG_ATTEMPTS, () -> motor.getConfigurator().apply(talonConfig, CONFIG_TIMEOUT_SECONDS));
        configFailedAlert.set(!status.isOK());

        absolutePosition = encoder.getAbsolutePosition();
        velocity = encoder.getVelocity();
        motorPosition = motor.getPosition();
        appliedVolts = motor.getMotorVoltage();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        PhoenixUtil.registerSignals(encoderBus, absolutePosition, velocity);
        PhoenixUtil.registerSignals(motorBus, motorPosition, appliedVolts, statorCurrent, supplyCurrent);
    }

    @Override
    public void updateInputs(IntakeDeployIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.isAllGood(motorPosition, appliedVolts, statorCurrent, supplyCurrent);
        inputs.encoderConnected = BaseStatusSignal.isAllGood(absolutePosition, velocity);
        inputs.positionRad = Units.rotationsToRadians(absolutePosition.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorAmps = statorCurrent.getValueAsDouble();
        inputs.supplyAmps = supplyCurrent.getValueAsDouble();
    }

    @Override
    public void setPositionGoal(double positionRad) {
        // With CANCoder as a remote sensor, we can directly set the position target in degrees without needing
        // to convert to motor rotations
        motor.setControl(positionRequest.withPosition(Units.radiansToRotations(positionRad)));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }

    @Override
    public void close() {
        configFailedAlert.close();
        motor.close();
        encoder.close();
    }
}
