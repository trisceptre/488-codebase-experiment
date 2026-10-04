package frc.team488.vision;

import frc.team488.LoopParticipant;
import frc.team488.Pose;
import frc.team488.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.util.Alert;

/** Reads every camera, filters each observation, and fuses the accepted ones into {@link Pose}. */
public final class Vision implements LoopParticipant, AutoCloseable {
    private final List<CameraInfo> cameras;
    private final List<AprilTagVisionIO> ios;
    private final List<AprilTagVisionIOInputsAutoLogged> inputs = new ArrayList<>();
    private final List<Debouncer> connectedDebouncers = new ArrayList<>();
    private final List<Alert> disconnectedAlerts = new ArrayList<>();
    private final double fieldLength;
    private final double fieldWidth;
    private final Pose pose;

    private final TunableNumber maxAmbiguity = new TunableNumber("Vision/Filter/MaxAmbiguity", 0.3);
    private final TunableNumber maxZError = new TunableNumber("Vision/Filter/MaxZErrorMeters", 0.75);
    private final TunableNumber minTagDistance = new TunableNumber("Vision/Filter/MinTagDistanceMeters", 0.5);
    private final TunableNumber maxSingleTagDistance =
            new TunableNumber("Vision/Filter/MaxSingleTagDistanceMeters", 1.0);
    private final TunableNumber maxMultiTagDistance = new TunableNumber("Vision/Filter/MaxMultiTagDistanceMeters", 5.0);
    private final TunableNumber linearStdDev = new TunableNumber("Vision/Filter/LinearStdDevBaselineMeters", 0.02);
    private final TunableNumber angularStdDev = new TunableNumber("Vision/Filter/AngularStdDevBaselineRad", 0.06);

    public Vision(List<CameraInfo> cameras, List<AprilTagVisionIO> ios, Field field, Pose pose) {
        if (cameras.size() != ios.size()) {
            throw new IllegalArgumentException("one IO per camera required");
        }
        this.cameras = List.copyOf(cameras);
        this.ios = List.copyOf(ios);
        this.fieldLength = field.getFieldLength();
        this.fieldWidth = field.getFieldWidth();
        this.pose = pose;
        for (CameraInfo camera : cameras) {
            inputs.add(new AprilTagVisionIOInputsAutoLogged());
            connectedDebouncers.add(new Debouncer(0.5, Debouncer.DebounceType.FALLING));
            disconnectedAlerts.add(new Alert(
                    "Vision/" + camera.name() + "/Disconnected",
                    camera.name() + " camera disconnected",
                    Alert.Level.MEDIUM));
        }
    }

    @Override
    public void updateInputs() {
        var settings = new VisionFilter.Settings(
                maxAmbiguity.get(),
                maxZError.get(),
                minTagDistance.get(),
                maxSingleTagDistance.get(),
                maxMultiTagDistance.get(),
                linearStdDev.get(),
                angularStdDev.get());
        for (int i = 0; i < cameras.size(); i++) {
            CameraInfo camera = cameras.get(i);
            var cameraInputs = inputs.get(i);
            ios.get(i).updateInputs(cameraInputs);
            Logger.processInputs("Vision/" + camera.name(), cameraInputs);
            disconnectedAlerts.get(i).set(!connectedDebouncers.get(i).calculate(cameraInputs.connected));

            List<Pose3d> accepted = new ArrayList<>();
            List<Pose3d> rejected = new ArrayList<>();
            List<String> reasons = new ArrayList<>();
            for (VisionPoseObservation observation : cameraInputs.observations) {
                var rejection = VisionFilter.check(observation, fieldLength, fieldWidth, settings);
                if (rejection == VisionFilter.Rejection.NONE) {
                    pose.addVisionMeasurement(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VisionFilter.stdDevs(observation, camera.stdDevFactor(), settings));
                    accepted.add(observation.pose());
                } else {
                    rejected.add(observation.pose());
                    reasons.add(rejection.name());
                }
            }
            String prefix = "Vision/" + camera.name();
            Logger.recordOutput(prefix + "/Accepted", accepted.toArray(new Pose3d[0]));
            Logger.recordOutput(prefix + "/Rejected", rejected.toArray(new Pose3d[0]));
            Logger.recordOutput(prefix + "/RejectionReasons", reasons.toArray(new String[0]));
        }
    }

    @Override
    public void applyOutputs() {}

    @Override
    public void close() {
        disconnectedAlerts.forEach(Alert::close);
    }
}
