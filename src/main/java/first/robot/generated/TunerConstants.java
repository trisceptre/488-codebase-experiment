package first.robot.generated;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

public class TunerConstants {
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(1.91)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final Slot0Configs driveGains =
            new Slot0Configs().withKP(0.2).withKV(0.124);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
    private static final Current kSlipCurrent = Amps.of(120);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(45))
                    .withStatorCurrentLimitEnable(true));
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    public static final CANBus kCANBus = new CANBus("*");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.9);

    private static final double kCoupleRatio = 0.0;
    private static final double kDriveGearRatio = 5.40; // Documented value for WCP x2i with X3 12t gears.
    private static final double kSteerGearRatio = 12.1; // Documented value for WCP x2i.
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 56;

    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.035);
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants().withNetwork(kCANBus).withPigeon2Id(kPigeonId);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            ConstantCreator = new SwerveModuleConstantsFactory<
                            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(kWheelRadius)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kSpeedAt12Volts)
                    .withDriveMotorType(kDriveMotorType)
                    .withSteerMotorType(kSteerMotorType)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(encoderInitialConfigs)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    private static final int kFrontLeftDriveMotorId = 30;
    private static final int kFrontLeftSteerMotorId = 31;
    private static final int kFrontLeftEncoderId = 53;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final Distance kFrontLeftXPos = Inches.of(11);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    private static final int kFrontRightDriveMotorId = 38;
    private static final int kFrontRightSteerMotorId = 39;
    private static final int kFrontRightEncoderId = 54;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.0);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;
    private static final Distance kFrontRightXPos = Inches.of(11);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    private static final int kBackLeftDriveMotorId = 28;
    private static final int kBackLeftSteerMotorId = 29;
    private static final int kBackLeftEncoderId = 52;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;
    private static final Distance kBackLeftXPos = Inches.of(-11);
    private static final Distance kBackLeftYPos = Inches.of(10);

    private static final int kBackRightDriveMotorId = 20;
    private static final int kBackRightSteerMotorId = 21;
    private static final int kBackRightEncoderId = 51;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.0);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;
    private static final Distance kBackRightXPos = Inches.of(-11);
    private static final Distance kBackRightYPos = Inches.of(-10);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft = ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightEncoderInverted);
}
