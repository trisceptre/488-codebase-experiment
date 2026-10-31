package first.robot.subsystems.drive;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import first.robot.LoopParticipant;
import first.robot.RobotState;
import first.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class Drive implements Subsystem, LoopParticipant, AutoCloseable {
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;
    private static final double WHEEL_RADIUS_SETTLE_SECONDS = 1.0;

    private static final class WheelRadiusState {
        double startTime;
        double[] startDistances;
        Rotation2d lastYaw;
        double gyroDelta;
    }

    private enum Request {
        IDLE,
        VELOCITY,
        X
    }

    private final SwerveConfig config;
    private final DriveIO io;
    private final RobotState robotState;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final HeadingModule heading;
    private final SwerveDriveKinematics kinematics;
    private final Debouncer[] moduleDebouncers;
    private final Alert[] moduleAlerts;
    private final Alert gyroAlert = new Alert("Pigeon disconnected: heading control off", Alert.AlertType.kError);
    private final Alert droppedAlert = new Alert("Odometry samples dropped (loop stalled)", Alert.AlertType.kWarning);

    private final LoggedTunableNumber translationDeadband =
            new LoggedTunableNumber("Drive/Joystick/TranslationDeadband", 0.15);
    private final LoggedTunableNumber rotationDeadband =
            new LoggedTunableNumber("Drive/Joystick/RotationDeadband", 0.05);
    private final LoggedTunableNumber headingStickThreshold =
            new LoggedTunableNumber("Drive/Joystick/HeadingStickThreshold", 0.5);
    private final LoggedTunableNumber precisionTranslation =
            new LoggedTunableNumber("Drive/Joystick/PrecisionTranslation", 0.1);
    private final LoggedTunableNumber precisionRotation =
            new LoggedTunableNumber("Drive/Joystick/PrecisionRotation", 0.2);
    private final LoggedTunableNumber drivingPowerScale =
            new LoggedTunableNumber("Drive/Joystick/DrivingPowerScale", 1.0);
    private final LoggedTunableNumber turningPowerScale =
            new LoggedTunableNumber("Drive/Joystick/TurningPowerScale", 1.0);
    private final LoggedTunableNumber maxTargetSpeed =
            new LoggedTunableNumber("Drive/MaxTargetSpeedMetersPerSecond", 4.5);
    private final LoggedTunableNumber maxTargetTurnRate = new LoggedTunableNumber("Drive/MaxTargetTurnRate", 8.0);
    private final LoggedTunableNumber headingKp = new LoggedTunableNumber("Drive/Heading/kP", 5.0);
    private final LoggedTunableNumber headingKd = new LoggedTunableNumber("Drive/Heading/kD", 0.0);
    private final LoggedTunableNumber headingToleranceDeg = new LoggedTunableNumber("Drive/Heading/ToleranceDeg", 2.0);

    private Request request = Request.IDLE;
    private ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    private boolean modulesHealthy = true;
    private long lastDropped = 0;
    private double[] lastModuleDistancesMeters;
    private Rotation2d lastGyroYaw = Rotation2d.kZero;
    private double wheelRadiusEstimateMeters = 0.0;

    public Drive(SwerveConfig config, DriveIO io, RobotState robotState) {
        this.config = config;
        this.io = io;
        this.robotState = robotState;
        this.kinematics = config.kinematics();
        this.heading = new HeadingModule(headingKp.get(), headingKd.get(), maxTargetTurnRateRadPerSec());
        int n = config.moduleCount();
        lastModuleDistancesMeters = new double[n];
        moduleDebouncers = new Debouncer[n];
        moduleAlerts = new Alert[n];
        for (int j = 0; j < n; j++) {
            String label = config.moduleLabels().get(j);
            moduleDebouncers[j] = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
            moduleAlerts[j] = new Alert(label + " module device disconnected", Alert.AlertType.kError);
        }
        register();
        setDefaultCommand(idle());
    }

    @Override
    public String getName() {
        return "Drive";
    }

    public Command idle() {
        return run(this::stop).withName("Drive.Idle");
    }

    public Command joystickDrive(JoystickInputs sticks, BooleanSupplier precision) {
        return run(() -> {
                    var intent = shape(sticks, precision.getAsBoolean());
                    driveFraction(intent, stickOmega(intent));
                })
                .withName("Drive.JoystickDrive");
    }

    public Command joystickDriveAtAngle(JoystickInputs sticks, Supplier<Rotation2d> angle, BooleanSupplier aim) {
        return run(() -> {
                    var intent = shape(sticks, false);
                    double omega;
                    if (aim.getAsBoolean() && canControlHeading()) {
                        omega = heading.calculate(robotState.getRotation(), angle.get());
                    } else {
                        omega = stickOmega(intent);
                    }
                    driveFraction(intent, omega);
                })
                .withName("Drive.JoystickDriveAtAngle");
    }

    public Command stopWithX() {
        // Turns the wheels into an X shape so it is harder to shove our robot
        return run(() -> request = Request.X).withName("Drive.StopWithX");
    }

    public Command wheelRadiusCharacterization() {
        var limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        var state = new WheelRadiusState();
        return runOnce(() -> {
                    limiter.reset(0.0);
                    state.startTime = Timer.getTimestamp();
                    state.startDistances = null;
                    state.lastYaw = lastGyroYaw;
                    state.gyroDelta = 0.0;
                })
                .andThen(run(() -> {
                    runFieldVelocity(new ChassisSpeeds(0.0, 0.0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)));
                    if (Timer.getTimestamp() - state.startTime >= WHEEL_RADIUS_SETTLE_SECONDS) {
                        if (state.startDistances == null) {
                            state.startDistances = lastModuleDistancesMeters.clone();
                        } else {
                            state.gyroDelta +=
                                    Math.abs(lastGyroYaw.minus(state.lastYaw).getRadians());
                            updateWheelRadiusEstimate(state.startDistances, state.gyroDelta);
                        }
                        state.lastYaw = lastGyroYaw;
                    }
                }))
                .withName("Drive.WheelRadiusCharacterization");
    }

    /**
     * Set the target movement speed and rotation, rotating around the center of the
     * robot.
     */
    public void runFieldVelocity(ChassisSpeeds velocity) {
        request = Request.VELOCITY;
        fieldVelocity = velocity;
    }

    public void runRobotVelocity(ChassisSpeeds velocity) {
        runFieldVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(velocity, robotState.getRotation()));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        int n = config.moduleCount();
        var states = new SwerveModuleState[n];
        for (int j = 0; j < n; j++) {
            states[j] = new SwerveModuleState(
                    j < inputs.moduleSpeedsMetersPerSec.length ? inputs.moduleSpeedsMetersPerSec[j] : 0.0,
                    Rotation2d.fromRadians(j < inputs.moduleAnglesRad.length ? inputs.moduleAnglesRad[j] : 0.0));
        }
        return kinematics.toChassisSpeeds(states);
    }

    public void resetPose(Pose2d pose) {
        robotState.resetPose(pose);
        io.resetPose(pose);
    }

    public void stop() {
        request = Request.IDLE;
        heading.reset();
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return config.maxLinearSpeedMetersPerSec();
    }

    public double getMaxAngularSpeedRadPerSec() {
        return config.maxAngularSpeedRadPerSec();
    }

    public double getWheelRadiusEstimateMeters() {
        return wheelRadiusEstimateMeters;
    }

    public boolean atHeading() {
        return heading.atSetpoint();
    }

    public boolean isHealthy() {
        return modulesHealthy && inputs.gyroConnected;
    }

    public boolean canControlHeading() {
        return inputs.gyroConnected;
    }

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
                lastModuleDistancesMeters[j] = positions[j].distanceMeters;
            }
            lastGyroYaw = Rotation2d.fromRadians(inputs.odometryYawRad[i]);
            robotState.addOdometryObservation(
                    new RobotState.OdometryObservation(inputs.odometryTimestamps[i], positions, lastGyroYaw));
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
        heading.setMaxOmega(maxTargetTurnRateRadPerSec());
        heading.setTolerance(Rotation2d.fromDegrees(headingToleranceDeg.get()));
    }

    @Override
    public void applyOutputs() {
        Request effective = edu.wpi.first.wpilibj.RobotState.isDisabled() ? Request.IDLE : request;
        switch (effective) {
            case IDLE -> io.idle();
            case X -> io.stopWithX();
            // rotate the translation vector into the robot coordinate frame
            case VELOCITY ->
                io.setRobotVelocity(limitWheelSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(fieldVelocity, robotState.getRotation())));
        }
        Logger.recordOutput("Drive/Request", effective.name());
        Logger.recordOutput("Drive/FieldVelocity", new double[] {
            fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond, fieldVelocity.omegaRadiansPerSecond
        });
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

    private void updateWheelRadiusEstimate(double[] startDistances, double gyroDelta) {
        double wheelDeltaMeters = 0.0;
        for (int j = 0; j < startDistances.length; j++) {
            wheelDeltaMeters += Math.abs(lastModuleDistancesMeters[j] - startDistances[j]) / startDistances.length;
        }
        if (wheelDeltaMeters > 0.0) {
            wheelRadiusEstimateMeters =
                    gyroDelta * config.driveBaseRadiusMeters() * config.wheelRadiusMeters() / wheelDeltaMeters;
        }
        Logger.recordOutput("Drive/WheelRadiusCharacterization/GyroDeltaRad", gyroDelta);
        Logger.recordOutput("Drive/WheelRadiusCharacterization/WheelDeltaMeters", wheelDeltaMeters);
        Logger.recordOutput("Drive/WheelRadiusCharacterization/RadiusMeters", wheelRadiusEstimateMeters);
    }

    private JoystickShaping.Intent shape(JoystickInputs sticks, boolean precision) {
        var settings = new JoystickShaping.Settings(
                translationDeadband.get(),
                rotationDeadband.get(),
                headingStickThreshold.get(),
                precisionTranslation.get(),
                precisionRotation.get());
        boolean red = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
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

    private double stickOmega(JoystickShaping.Intent intent) {
        if (intent.heading().isPresent() && canControlHeading()) {
            return heading.calculate(robotState.getRotation(), intent.heading().get());
        }
        heading.reset();
        return intent.omegaFraction() * maxTargetTurnRateRadPerSec();
    }

    private void driveFraction(JoystickShaping.Intent intent, double omegaRadPerSec) {
        double speed = maxTargetSpeedMetersPerSec() * drivingPowerScale.get();
        runFieldVelocity(new ChassisSpeeds(
                intent.vxFraction() * speed, intent.vyFraction() * speed, omegaRadPerSec * turningPowerScale.get()));
    }

    private double maxTargetSpeedMetersPerSec() {
        return Math.min(maxTargetSpeed.get(), config.maxLinearSpeedMetersPerSec());
    }

    private double maxTargetTurnRateRadPerSec() {
        return Math.min(maxTargetTurnRate.get(), config.maxAngularSpeedRadPerSec());
    }

    private ChassisSpeeds limitWheelSpeeds(ChassisSpeeds robotVelocity) {
        double fastest = 0.0;
        for (SwerveModuleState state : kinematics.toSwerveModuleStates(robotVelocity)) {
            fastest = Math.max(fastest, Math.abs(state.speedMetersPerSecond));
        }
        double limit = maxTargetSpeedMetersPerSec();
        return fastest > limit ? robotVelocity.times(limit / fastest) : robotVelocity;
    }

    private static boolean connected(boolean[] flags, int index) {
        return index >= flags.length || flags[index];
    }
}
