package first.lib.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.lib.util.PhoenixUtil;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;

public class MotorIOTalonFX implements MotorIO, AutoCloseable {
    private static final int CONFIG_ATTEMPTS = 3;
    private static final double CONFIG_TIMEOUT_SECONDS = 0.25;
    private static final double RUNTIME_TIMEOUT_SECONDS = 0.05;

    private final TalonFX leader;
    private final List<TalonFX> followers = new ArrayList<>();
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;
    private final List<StatusSignal<Current>> followerSupplyCurrents = new ArrayList<>();
    private final List<StatusSignal<Temperature>> followerTemperatures = new ArrayList<>();

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final boolean useMotionMagic;
    private final Slot0Configs slot0;

    private final Alert configFailedAlert;

    public MotorIOTalonFX(MotorConfig config) {
        TalonFXConfiguration talonConfig = toTalonConfig(config);
        slot0 = talonConfig.Slot0;
        useMotionMagic = config.shaping().motionMagic().isPresent();
        List<String> configFailures = new ArrayList<>();

        CANBus leaderBus = canBus(config.leader());
        leader = new TalonFX(config.leader().id(), leaderBus);
        applyConfig(leader, talonConfig, config.name(), configFailures);
        position = leader.getPosition();
        velocity = leader.getVelocity();
        appliedVolts = leader.getMotorVoltage();
        supplyCurrent = leader.getSupplyCurrent();
        statorCurrent = leader.getStatorCurrent();
        temperature = leader.getDeviceTemp();
        PhoenixUtil.registerSignals(
                leaderBus, position, velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);

        for (MotorConfig.Follower follower : config.followers()) {
            CANBus bus = canBus(follower.id());
            TalonFX talon = new TalonFX(follower.id().id(), bus);
            applyConfig(
                    talon,
                    talonConfig,
                    config.name() + " follower " + follower.id().id(),
                    configFailures);
            talon.setControl(new Follower(
                    config.leader().id(),
                    follower.opposeLeader() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            followers.add(talon);
            StatusSignal<Current> current = talon.getSupplyCurrent();
            StatusSignal<Temperature> temp = talon.getDeviceTemp();
            followerSupplyCurrents.add(current);
            followerTemperatures.add(temp);
            PhoenixUtil.registerSignals(bus, current, temp);
        }

        configFailedAlert = new Alert(
                config.name() + "/ConfigFailed",
                "Config failed: " + String.join(", ", configFailures),
                Alert.Level.HIGH);
        configFailedAlert.set(!configFailures.isEmpty());
    }

    @Override
    public void close() {
        configFailedAlert.close();
        followers.forEach(TalonFX::close);
        leader.close();
    }

    public static CANBus canBus(CANId id) {
        return id.bus().isEmpty() ? new CANBus() : new CANBus(id.bus());
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected =
                BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyAmps = supplyCurrent.getValueAsDouble();
        inputs.statorAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();

        int count = followers.size();
        inputs.followerConnected = new boolean[count];
        inputs.followerSupplyAmps = new double[count];
        inputs.followerTempCelsius = new double[count];
        for (int i = 0; i < count; i++) {
            inputs.followerConnected[i] =
                    BaseStatusSignal.isAllGood(followerSupplyCurrents.get(i), followerTemperatures.get(i));
            inputs.followerSupplyAmps[i] = followerSupplyCurrents.get(i).getValueAsDouble();
            inputs.followerTempCelsius[i] = followerTemperatures.get(i).getValueAsDouble();
        }
    }

    @Override
    public void applyOutputs(Outputs outputs) {
        switch (outputs.mode) {
            case NEUTRAL -> leader.setControl(neutralRequest);
            case VOLTAGE ->
                leader.setControl(
                        voltageRequest.withOutput(outputs.value).withIgnoreSoftwareLimits(outputs.ignoreSoftLimits));
            case VELOCITY -> leader.setControl(velocityRequest.withVelocity(Units.radiansToRotations(outputs.value)));
            case POSITION -> {
                double rotations = Units.radiansToRotations(outputs.value);
                leader.setControl(
                        useMotionMagic
                                ? motionMagicRequest.withPosition(rotations)
                                : positionRequest.withPosition(rotations));
            }
        }
    }

    @Override
    public boolean setGains(double kP, double kD) {
        slot0.kP = kP;
        slot0.kD = kD;
        return leader.getConfigurator().apply(slot0, RUNTIME_TIMEOUT_SECONDS).isOK();
    }

    @Override
    public boolean setPosition(double positionRad) {
        return leader.setPosition(Units.radiansToRotations(positionRad), RUNTIME_TIMEOUT_SECONDS)
                .isOK();
    }

    static TalonFXConfiguration toTalonConfig(MotorConfig config) {
        var talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.Inverted =
                config.inverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = config.brakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talonConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismRatio();
        MotorConfig.Gains gains = config.gains();
        talonConfig.Slot0.kP = gains.kP();
        talonConfig.Slot0.kD = gains.kD();
        talonConfig.Slot0.kS = gains.kS();
        talonConfig.Slot0.kV = gains.kV();
        talonConfig.Slot0.kA = gains.kA();
        talonConfig.Slot0.kG = gains.gravityType() == MotorConfig.GravityType.NONE ? 0.0 : gains.kG();
        talonConfig.Slot0.GravityType = gains.gravityType() == MotorConfig.GravityType.ARM
                ? GravityTypeValue.Arm_Cosine
                : GravityTypeValue.Elevator_Static;
        talonConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimits().supplyAmps();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = config.currentLimits().statorAmps();
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.currentLimits().supplyLower().ifPresent(lower -> {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = lower.amps();
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = lower.seconds();
        });
        double ramp = config.shaping().rampSeconds();
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ramp;
        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ramp;
        config.shaping().motionMagic().ifPresent(profile -> {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(profile.cruiseRadPerSec());
            talonConfig.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(profile.accelRadPerSecSq());
        });
        config.softLimits().ifPresent(limits -> {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(limits.minRad());
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(limits.maxRad());
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        });
        return talonConfig;
    }

    private static void applyConfig(
            TalonFX talon, TalonFXConfiguration talonConfig, String label, List<String> failures) {
        StatusCode status = PhoenixUtil.tryUntilOk(
                CONFIG_ATTEMPTS, () -> talon.getConfigurator().apply(talonConfig, CONFIG_TIMEOUT_SECONDS));
        if (!status.isOK()) {
            failures.add(label + " (" + status + ")");
        }
    }
}
