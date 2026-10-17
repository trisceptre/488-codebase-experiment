package first.robot.game;

import static edu.wpi.first.units.Units.RPM;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import first.robot.util.LoggedTunableNumber;
import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public final class ShotCalculator {
    private static final Transform2d SHOOTER_OFFSET =
            new Transform2d(Units.inchesToMeters(-23.5), 0.0, Rotation2d.kZero);

    public record Shot(Rotation2d heading, AngularVelocity shooterSpeed, double hoodRatio) {
        public static final Shot NONE = new Shot(Rotation2d.kZero, RPM.of(0.0), 0.0);
    }

    public record Preset(AngularVelocity shooterSpeed, double hoodRatio) {}

    public enum PresetDistance {
        // Note: robot radius corrections below are educated guesses but don't matter
        // precisely here.
        NEAR(0.94 + 0.34, 2725, 0.0),
        TOWER_CLOSE(3.52 - 0.60, 3400, 0.2),
        TOWER_FAR(2.64 + 0.34, 3900, 0.0),
        TRENCH(2.74 + 0.34, 4100, 0.0),
        CORNER(6.27 - 0.6, 3750, 1.0);

        final double distanceMeters;

        final double defaultRpm;
        final double defaultHoodRatio;

        PresetDistance(double distanceMeters, double defaultRpm, double defaultHoodRatio) {
            this.distanceMeters = distanceMeters;
            this.defaultRpm = defaultRpm;
            this.defaultHoodRatio = defaultHoodRatio;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    record TableEntry(
            @JsonProperty("distance") double distance,
            @JsonProperty("servo") double servo,
            @JsonProperty("RPM") double rpm) {}

    private record PresetTunables(LoggedTunableNumber rpm, LoggedTunableNumber hoodRatio) {}

    private static final class Table {
        private final InterpolatingDoubleTreeMap rpm = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap hoodRatio = new InterpolatingDoubleTreeMap();
        private double maxDistance = Double.NEGATIVE_INFINITY;

        Table(List<TableEntry> entries) {
            for (TableEntry entry : entries) {
                rpm.put(entry.distance(), entry.rpm());
                hoodRatio.put(entry.distance(), entry.servo());
                maxDistance = Math.max(maxDistance, entry.distance());
            }
        }

        boolean covers(double distance) {
            return distance <= maxDistance;
        }
    }

    private final AprilTagFieldLayout field;
    private final Table table;
    private final Table zeroHoodTable;
    private final Map<PresetDistance, PresetTunables> presets = new EnumMap<>(PresetDistance.class);
    private final LoggedTunableNumber version = new LoggedTunableNumber("ShotCalculator/Version", 3);
    private final LoggedTunableNumber distanceOffsetMeters =
            new LoggedTunableNumber("ShotCalculator/V3DistanceOffsetMeters", 0.45);
    private final LoggedTunableNumber allianceZoneAimInterpolation =
            new LoggedTunableNumber("ShotCalculator/AllianceZoneAimInterpolation", 0.5);

    public ShotCalculator(AprilTagFieldLayout field) {
        this(
                field,
                loadTable(new File(Filesystem.getDeployDirectory(), "trajectories.json")),
                loadTable(new File(Filesystem.getDeployDirectory(), "trajectories_0_hood.json")));
    }

    ShotCalculator(AprilTagFieldLayout field, List<TableEntry> table, List<TableEntry> zeroHoodTable) {
        this.field = field;
        this.table = new Table(table);
        this.zeroHoodTable = new Table(zeroHoodTable);
        for (PresetDistance preset : PresetDistance.values()) {
            presets.put(
                    preset,
                    new PresetTunables(
                            new LoggedTunableNumber("ShotCalculator/Preset/" + preset + "/RPM", preset.defaultRpm),
                            new LoggedTunableNumber(
                                    "ShotCalculator/Preset/" + preset + "/HoodRatio", preset.defaultHoodRatio)));
        }
    }

    public Preset preset(PresetDistance distance) {
        var tunables = presets.get(distance);
        return new Preset(RPM.of(tunables.rpm().get()), tunables.hoodRatio().get());
    }

    public Shot hubShot(Pose2d robot, boolean zeroHood) {
        return shot(robot, Landmarks.allianceHub(field, alliance()), zeroHood);
    }

    public Shot allianceZoneShot(Pose2d robot, boolean zeroHood) {
        Alliance alliance = alliance();
        Translation2d trench =
                Landmarks.closestTrenchNeutralSide(field, alliance, robot).getTranslation();
        Translation2d target = Landmarks.allianceHub(field, alliance)
                .getTranslation()
                .interpolate(trench, allianceZoneAimInterpolation.get());
        return shot(robot, new Pose2d(target, Rotation2d.kZero), zeroHood);
    }

    private Shot shot(Pose2d robot, Pose2d target, boolean zeroHood) {
        return switch ((int) version.get()) {
            // Fixed shooter parameters. Should only be used when things to very wrong.
            case 1 -> new Shot(headingTo(robot, target), RPM.of(3800), 0.2);
            case 2 -> nearestPreset(robot, target);
            default -> tableShot(robot, target, zeroHood);
        };
    }

    // Look up known distances based on a preset distances rather than continuous
    // adjustment.
    private Shot nearestPreset(Pose2d robot, Pose2d target) {
        double distance = target.getTranslation().getDistance(robot.getTranslation());
        PresetDistance nearest = List.of(PresetDistance.values()).stream()
                .min(Comparator.comparingDouble(p -> Math.abs(distance - p.distanceMeters)))
                .orElse(PresetDistance.NEAR);
        var preset = preset(nearest);

        // Target rotation will probably be ignored or corrected if we're using manual
        // distance, but point towards where we think the hub
        return new Shot(headingTo(robot, target), preset.shooterSpeed(), preset.hoodRatio());
    }

    // Look up optimal shooting parameters based on current pose and shooting
    // target's pose.
    private Shot tableShot(Pose2d robot, Pose2d target, boolean zeroHood) {
        Rotation2d heading = headingTo(robot, target);
        Pose2d shooter = new Pose2d(robot.getTranslation(), heading).plus(SHOOTER_OFFSET);
        double distance = shooter.getTranslation().getDistance(target.getTranslation());
        double lookupDistance = distance + distanceOffsetMeters.get();
        Logger.recordOutput("ShotCalculator/DistanceMeters", distance);
        Logger.recordOutput("ShotCalculator/LookupDistanceMeters", lookupDistance);

        Table chosen = zeroHood ? zeroHoodTable : table;
        boolean inRange = chosen.covers(lookupDistance);
        Logger.recordOutput("ShotCalculator/InRange", inRange);
        if (!inRange) {
            return Shot.NONE;
        }
        return new Shot(heading, RPM.of(chosen.rpm.get(lookupDistance)), chosen.hoodRatio.get(lookupDistance));
    }

    private static Rotation2d headingTo(Pose2d robot, Pose2d target) {
        Translation2d toTarget = target.getTranslation().minus(robot.getTranslation());
        return toTarget.getNorm() < 0.01 ? robot.getRotation() : toTarget.getAngle();
    }

    static List<TableEntry> loadTable(File file) {
        try {
            return List.of(new ObjectMapper().readValue(file, TableEntry[].class));
        } catch (IOException e) {
            DriverStation.reportError("Could not load shot table " + file + ": " + e.getMessage(), false);
            return List.of();
        }
    }

    private static Alliance alliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
