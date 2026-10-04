package frc.team488.mechanisms.drive;

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
import java.util.List;

/**
 * The competition drivetrain, in the shape Tuner X generates. TODO(season): these values are the 2026
 * chassis (WCP x2i modules, TeamXbot2026's Contract2026) and haven't been run on SystemCore yet;
 * regenerate them in Tuner X for this season's robot.
 */
public final class DriveConstants {
    private DriveConstants() {}

    private static final CANBus CANIVORE = new CANBus("*"); // MEASURE: CANivore name on SystemCore

    private static final double DRIVE_GEAR_RATIO = 5.40; // x2i with X3 12t
    private static final double STEER_GEAR_RATIO = 12.1;
    private static final double COUPLE_RATIO = 0.0; // MEASURE: Tuner X can measure azimuth coupling
    private static final double SPEED_AT_12V_MPS = 5.9; // MEASURE: estimate for a Kraken X60 at 5.40:1

    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(1.91)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final Slot0Configs DRIVE_GAINS =
            new Slot0Configs().withKP(0.2).withKV(0.124);

    private static final TalonFXConfiguration DRIVE_INITIAL = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(45))
                    .withStatorCurrentLimitEnable(true));
    private static final TalonFXConfiguration STEER_INITIAL = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true));

    private static final SwerveDrivetrainConstants DRIVETRAIN =
            new SwerveDrivetrainConstants().withNetwork(CANIVORE).withPigeon2Id(56);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FACTORY = new SwerveModuleConstantsFactory<
                            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLE_RATIO)
                    .withWheelRadius(Inches.of(2))
                    .withSteerMotorGains(STEER_GAINS)
                    .withDriveMotorGains(DRIVE_GAINS)
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(120))
                    .withSpeedAt12Volts(MetersPerSecond.of(SPEED_AT_12V_MPS))
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(DRIVE_INITIAL)
                    .withSteerMotorInitialConfigs(STEER_INITIAL)
                    .withEncoderInitialConfigs(new CANcoderConfiguration())
                    .withSteerInertia(KilogramSquareMeters.of(0.01))
                    .withDriveInertia(KilogramSquareMeters.of(0.035))
                    .withSteerFrictionVoltage(Volts.of(0.2))
                    .withDriveFrictionVoltage(Volts.of(0.2));

    // MEASURE: CANcoder magnet offsets live on the devices. Read them in Tuner X and copy them here
    // before the first deploy, or the modules will be misaligned. Robot raises an alert while any is 0.
    private static final double FL_OFFSET_ROT = 0.0;
    private static final double FR_OFFSET_ROT = 0.0;
    private static final double RL_OFFSET_ROT = 0.0;
    private static final double RR_OFFSET_ROT = 0.0;

    private static final boolean INVERT_LEFT_DRIVE = false; // MEASURE: verify on the robot
    private static final boolean INVERT_RIGHT_DRIVE = true; // MEASURE: verify on the robot
    private static final boolean STEER_INVERTED = true;
    private static final boolean ENCODER_INVERTED = false;

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> module(
            int steerId, int driveId, int encoderId, double offsetRot, double xIn, double yIn, boolean invertDrive) {
        return FACTORY.createModuleConstants(
                steerId,
                driveId,
                encoderId,
                Rotations.of(offsetRot),
                Inches.of(xIn),
                Inches.of(yIn),
                invertDrive,
                STEER_INVERTED,
                ENCODER_INVERTED);
    }

    public static final SwerveConfig CONFIG = new SwerveConfig(
            DRIVETRAIN,
            List.of(
                    module(31, 30, 53, FL_OFFSET_ROT, 11, 10, INVERT_LEFT_DRIVE),
                    module(39, 38, 54, FR_OFFSET_ROT, 11, -10, INVERT_RIGHT_DRIVE),
                    module(29, 28, 52, RL_OFFSET_ROT, -11, 10, INVERT_LEFT_DRIVE),
                    module(21, 20, 51, RR_OFFSET_ROT, -11, -10, INVERT_RIGHT_DRIVE)),
            List.of("FrontLeft", "FrontRight", "RearLeft", "RearRight"),
            SPEED_AT_12V_MPS,
            // Max turn rate: top speed around the 14.87 in (0.378 m) module radius.
            SPEED_AT_12V_MPS / 0.378);
}
