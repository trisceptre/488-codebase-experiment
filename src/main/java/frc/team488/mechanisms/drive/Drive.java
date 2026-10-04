package frc.team488.mechanisms.drive;

import frc.team488.LoopParticipant;
import frc.team488.Pose;
import frc.team488.util.TunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.util.Alert;

/**
 * Swerve drive. Commands set a field-relative goal; {@link #applyOutputs()} converts it to
 * robot-relative using {@link Pose}'s heading, so there is one source of truth for heading.
 */
public final class Drive implements Mechanism, LoopParticipant, AutoCloseable {
    private enum Request {
        IDLE,
        VELOCITY,
        LOCK
    }

    private final SwerveConfig config;
    private final DriveIO io;
    private final Pose pose;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final HeadingModule heading;
    private final Debouncer[] moduleDebouncers;
    private final Alert[] moduleAlerts;
    private final Alert gyroAlert =
            new Alert("Drive/GyroDisconnected", "Pigeon disconnected: heading control off", Alert.Level.HIGH);
    private final Alert droppedAlert =
            new Alert("Drive/OdometryDropped", "Odometry samples dropped (loop stalled)", Alert.Level.MEDIUM);

    private final TunableNumber translationDeadband = new TunableNumber("Drive/Joystick/TranslationDeadband", 0.15);
    private final TunableNumber rotationDeadband = new TunableNumber("Drive/Joystick/RotationDeadband", 0.05);
    private final TunableNumber headingStickThreshold = new TunableNumber("Drive/Joystick/HeadingStickThreshold", 0.5);
    private final TunableNumber precisionTranslation = new TunableNumber("Drive/Joystick/PrecisionTranslation", 0.1);
    private final TunableNumber precisionRotation = new TunableNumber("Drive/Joystick/PrecisionRotation", 0.2);
    private final TunableNumber headingKp = new TunableNumber("Drive/Heading/kP", 5.0);
    private final TunableNumber headingKd = new TunableNumber("Drive/Heading/kD", 0.0);
    private final TunableNumber headingToleranceDeg = new TunableNumber("Drive/Heading/ToleranceDeg", 2.0);

    private Request request = Request.IDLE;
    private ChassisVelocities fieldVelocity = new ChassisVelocities();
    private boolean modulesHealthy = true;
    private long lastDropped = 0;

    public Drive(SwerveConfig config, DriveIO io, Pose pose) {
        this.config = config;
        this.io = io;
        this.pose = pose;
        this.heading = new HeadingModule(headingKp.get(), headingKd.get(), config.maxAngularRateRadPerSec());
        int n = config.moduleCount();
        moduleDebouncers = new Debouncer[n];
        moduleAlerts = new Alert[n];
        for (int j = 0; j < n; j++) {
            String label = config.moduleLabels().get(j);
            moduleDebouncers[j] = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
            moduleAlerts[j] = new Alert(
                    "Drive/" + label + "/Disconnected", label + " module device disconnected", Alert.Level.HIGH);
        }
        setDefaultCommand(idle());
    }

    @Override
    public String getName() {
        return "Drive";
    }

    // ---- Commands ----

    @Override
    public Command idle() {
        return run(coroutine -> {
                    stop();
                    coroutine.park();
                })
                .withPriority(Command.LOWEST_PRIORITY)
                .named("Drive.Idle");
    }

    /** 488's teleop scheme. Hold {@code precision} for slow, fine control. */
    public Command joystickDrive(JoystickInputs sticks, BooleanSupplier precision) {
        return runRepeatedly(() -> {
                    var intent = shape(sticks, precision.getAsBoolean());
                    double omega;
                    if (intent.heading().isPresent() && canControlHeading()) {
                        omega = heading.calculate(
                                pose.heading(), intent.heading().get());
                    } else {
                        heading.reset();
                        omega = intent.omegaFraction() * config.maxAngularRateRadPerSec();
                    }
                    driveFraction(intent, omega);
                })
                .named("Drive.JoystickDrive");
    }

    /** Driver translates; rotation always faces {@code target}. Runs until interrupted. */
    public Command aimAt(JoystickInputs sticks, Supplier<Translation2d> target) {
        return runRepeatedly(() -> {
                    var intent = shape(sticks, false);
                    double omega;
                    if (canControlHeading()) {
                        Translation2d toTarget = target.get().minus(pose.get().getTranslation());
                        // getAngle is empty when on top of the target; hold the current heading then.
                        omega = heading.calculate(
                                pose.heading(), toTarget.getAngle().orElse(pose.heading()));
                    } else {
                        heading.reset();
                        omega = intent.omegaFraction() * config.maxAngularRateRadPerSec();
                    }
                    driveFraction(intent, omega);
                })
                .named("Drive.AimAt");
    }

    /** The "X-lock" stance: wheels point inward to resist pushing. */
    public Command lockWheels() {
        return run(coroutine -> {
                    lock();
                    coroutine.park();
                })
                .named("Drive.LockWheels");
    }

    // ---- Goals and status ----

    public void setFieldVelocity(ChassisVelocities velocity) {
        request = Request.VELOCITY;
        fieldVelocity = velocity;
    }

    public void lock() {
        request = Request.LOCK;
    }

    public void stop() {
        request = Request.IDLE;
        heading.reset();
    }

    public boolean atHeading() {
        return heading.atHeading();
    }

    public boolean isHealthy() {
        return modulesHealthy && inputs.gyroConnected;
    }

    public boolean canControlHeading() {
        return inputs.gyroConnected;
    }

    // ---- Loop ----

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        int m = config.moduleCount();
        int samples = inputs.odometryTimestamps.length;
        for (int i = 0; i < samples; i++) {
            var positions = new SwerveModulePosition[m];
            for (int j = 0; j < m; j++) {
                positions[j] = new SwerveModulePosition(
                        inputs.odometryModuleDistancesMeters[i * m + j],
                        Rotation2d.fromRadians(inputs.odometryModuleAnglesRad[i * m + j]));
            }
            pose.addOdometrySample(
                    inputs.odometryTimestamps[i], Rotation2d.fromRadians(inputs.odometryYawRad[i]), positions);
        }

        boolean allHealthy = true;
        for (int j = 0; j < m; j++) {
            boolean ok = connected(inputs.driveConnected, j)
                    && connected(inputs.steerConnected, j)
                    && connected(inputs.encoderConnected, j);
            boolean debounced = moduleDebouncers[j].calculate(ok);
            moduleAlerts[j].set(!debounced);
            allHealthy &= debounced;
        }
        modulesHealthy = allHealthy;
        gyroAlert.set(!inputs.gyroConnected);
        droppedAlert.set(inputs.droppedOdometrySamples > lastDropped);
        lastDropped = inputs.droppedOdometrySamples;

        heading.setGains(headingKp.get(), headingKd.get());
        heading.setTolerance(Rotation2d.fromDegrees(headingToleranceDeg.get()));
    }

    @Override
    public void applyOutputs() {
        Request effective = RobotState.isDisabled() ? Request.IDLE : request;
        switch (effective) {
            case IDLE -> io.idle();
            case LOCK -> io.lockWheels();
            case VELOCITY -> io.setRobotVelocity(fieldVelocity.toRobotRelative(pose.heading()));
        }
        Logger.recordOutput("Drive/Request", effective.name());
        Logger.recordOutput(
                "Drive/FieldVelocity", new double[] {fieldVelocity.vx, fieldVelocity.vy, fieldVelocity.omega});
        Logger.recordOutput("Drive/AtHeading", atHeading());
    }

    @Override
    public void close() {
        for (Alert alert : moduleAlerts) {
            alert.close();
        }
        gyroAlert.close();
        droppedAlert.close();
    }

    private JoystickShaping.Intent shape(JoystickInputs sticks, boolean precision) {
        var settings = new JoystickShaping.Settings(
                translationDeadband.get(),
                rotationDeadband.get(),
                headingStickThreshold.get(),
                precisionTranslation.get(),
                precisionRotation.get());
        boolean red = MatchState.getAlliance().map(a -> a == Alliance.RED).orElse(false);
        return JoystickShaping.shape(
                sticks.leftX().getAsDouble(),
                sticks.leftY().getAsDouble(),
                sticks.leftTrigger().getAsDouble(),
                sticks.rightTrigger().getAsDouble(),
                sticks.rightX().getAsDouble(),
                sticks.rightY().getAsDouble(),
                red,
                precision,
                settings);
    }

    private void driveFraction(JoystickShaping.Intent intent, double omegaRadPerSec) {
        setFieldVelocity(new ChassisVelocities(
                intent.vxFraction() * config.maxSpeedMetersPerSec(),
                intent.vyFraction() * config.maxSpeedMetersPerSec(),
                omegaRadPerSec));
    }

    private static boolean connected(boolean[] flags, int index) {
        return index >= flags.length || flags[index];
    }
}
