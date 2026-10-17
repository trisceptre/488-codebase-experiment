package first.robot.game;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public final class Landmarks {
    private Landmarks() {}

    // General field
    public static final Pose2d OVERALL_CENTER = new Pose2d(8.270, 4.040, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_OUTPOST_SIDE_TRENCH_ENTRANCE = new Pose2d(5.540, .650, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_DEPOT_SIDE_TRENCH_ENTRANCE = new Pose2d(5.540, 7.390, Rotation2d.fromDegrees(180));

    // Starting on blue alliance towards outpost
    public static final Pose2d BLUE_START_TRENCH_TO_OUTPOST = new Pose2d(3.57, .650, Rotation2d.fromDegrees(0.00));
    public static final Pose2d BLUE_START_BUMP_TO_OUTPOST = new Pose2d(3.57, 2.3, Rotation2d.fromDegrees(0));

    // Starting on blue alliance towards depot
    public static final Pose2d BLUE_START_TRENCH_TO_DEPOT = new Pose2d(3.57, 7.390, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_START_BUMP_TO_DEPOT = new Pose2d(3.57, 5.980, Rotation2d.fromDegrees(0));

    // Starting on blue alliance towards neutral area
    public static final Pose2d BLUE_START_BUMP_TO_NEUTRAL_AREA_OUTPOST_SIDE =
            new Pose2d(4.480, .65, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_START_TRENCH_TO_NEUTRAL_AREA_OUTPOST_SIDE =
            new Pose2d(4.480, 2.6, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_START_BUMP_TO_NEUTRAL_AREA_DEPOT_SIDE =
            new Pose2d(3.580, 5.94, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_START_TRENCH_TO_NEUTRAL_AREA_DEPOT_SIDE =
            new Pose2d(4.480, 7.390, Rotation2d.fromDegrees(180));

    // Blue Depot
    public static final Pose2d BLUE_DEPOT_COLLECT_CENTER = new Pose2d(1.180, 5.940, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_DEPOT_WALL_SIDE = new Pose2d(0.460, 6.980, Rotation2d.fromDegrees(270));
    public static final Pose2d BLUE_DEPOT_TOWER_SIDE = new Pose2d(0.460, 4.950, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_DEPOT_CENTER = new Pose2d(0.270, 5.940, Rotation2d.fromDegrees(0));

    // Blue Outpost
    public static final Pose2d BLUE_OUTPOST = new Pose2d(0, 0.650, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_OUTPOST_PARK = new Pose2d(0.350, 0.650, Rotation2d.fromDegrees(0));

    // Blue Tower
    public static final Pose2d BLUE_CLIMB_OUTPOST_EDGE = new Pose2d(1.510, 2.780, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CLIMB_MIDDLE_OUTPOST_SIDE = new Pose2d(1.510, 3.120, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CLIMB_CENTER = new Pose2d(1.510, 3.750, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CLIMB_MIDDLE_DEPOT_SIDE = new Pose2d(1.510, 4.380, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CLIMB_DEPOT_SIDE_EDGE = new Pose2d(1.510, 4.650, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_TOWER_CENTER = new Pose2d(.48, 3.810, Rotation2d.fromDegrees(0));

    // Field of fuel (For auto)
    public static final Pose2d BLUE_OUTPOST_SIDE_FUEL_FIELD_COLLECTION_START =
            new Pose2d(7.790, 1.640, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_DEPOT_SIDE_FUEL_FIELD_COLLECTION_START =
            new Pose2d(7.790, 6.800, Rotation2d.fromDegrees(270));

    private static final double HALF_WIDTH_BALL_PIT = Units.inchesToMeters(35.95);
    private static final double HALF_HEIGHT_BALL_PIT = Units.inchesToMeters(90.95);
    private static final Transform2d[] BALL_PIT_CORNERS = {
        new Transform2d(HALF_WIDTH_BALL_PIT, HALF_HEIGHT_BALL_PIT, Rotation2d.kZero),
        new Transform2d(-HALF_WIDTH_BALL_PIT, HALF_HEIGHT_BALL_PIT, Rotation2d.kZero),
        new Transform2d(HALF_WIDTH_BALL_PIT, -HALF_HEIGHT_BALL_PIT, Rotation2d.kZero),
        new Transform2d(-HALF_WIDTH_BALL_PIT, -HALF_HEIGHT_BALL_PIT, Rotation2d.kZero),
    };

    // AprilTag IDs
    public static final int RED_CENTER_HUB_NEUTRAL_SIDE_ID = 4;
    public static final int RED_CENTER_HUB_DRIVER_SIDE_ID = 10;
    public static final int BLUE_CENTER_HUB_NEUTRAL_SIDE_ID = 20;
    public static final int BLUE_CENTER_HUB_DRIVER_SIDE_ID = 26;
    public static final int BLUE_TRENCH_DRIVER_DEPOT_SIDE_ID = 23;
    public static final int RED_TRENCH_DRIVER_DEPOT_SIDE_ID = 7;
    public static final int BLUE_TRENCH_NEUTRAL_DEPOT_SIDE_ID = 22;
    public static final int RED_TRENCH_NEUTRAL_DEPOT_SIDE_ID = 6;
    public static final int BLUE_TRENCH_NEUTRAL_OUTPOST_SIDE_ID = 17;
    public static final int RED_TRENCH_NEUTRAL_OUTPOST_SIDE_ID = 1;
    public static final int BLUE_TRENCH_DRIVER_OUTPOST_SIDE_ID = 28;
    public static final int RED_TRENCH_DRIVER_OUTPOST_SIDE_ID = 12;
    public static final int BLUE_OUTPOST_ID = 29;
    public static final int RED_OUTPOST_ID = 13;

    private static boolean blue(Alliance alliance) {
        return alliance == Alliance.Blue;
    }

    public static List<Integer> allianceHubCenterIds(Alliance alliance) {
        return blue(alliance)
                ? List.of(BLUE_CENTER_HUB_NEUTRAL_SIDE_ID, BLUE_CENTER_HUB_DRIVER_SIDE_ID)
                : List.of(RED_CENTER_HUB_NEUTRAL_SIDE_ID, RED_CENTER_HUB_DRIVER_SIDE_ID);
    }

    public static int allianceHubNeutralSideId(Alliance alliance) {
        return blue(alliance) ? BLUE_CENTER_HUB_NEUTRAL_SIDE_ID : RED_CENTER_HUB_NEUTRAL_SIDE_ID;
    }

    public static int trenchNeutralDepotSideId(Alliance alliance) {
        return blue(alliance) ? BLUE_TRENCH_NEUTRAL_DEPOT_SIDE_ID : RED_TRENCH_NEUTRAL_DEPOT_SIDE_ID;
    }

    public static int trenchNeutralOutpostSideId(Alliance alliance) {
        return blue(alliance) ? BLUE_TRENCH_NEUTRAL_OUTPOST_SIDE_ID : RED_TRENCH_NEUTRAL_OUTPOST_SIDE_ID;
    }

    public static int trenchDriverDepotSideId(Alliance alliance) {
        return blue(alliance) ? BLUE_TRENCH_DRIVER_DEPOT_SIDE_ID : RED_TRENCH_DRIVER_DEPOT_SIDE_ID;
    }

    public static int outpostId(Alliance alliance) {
        return blue(alliance) ? BLUE_OUTPOST_ID : RED_OUTPOST_ID;
    }

    // Get the average pose of the alliance hub using april tags
    public static Pose2d allianceHub(AprilTagFieldLayout layout, Alliance alliance) {
        List<Pose2d> tags = tagPoses(layout, allianceHubCenterIds(alliance));
        double x = tags.stream().mapToDouble(Pose2d::getX).average().orElse(0.0);
        double y = tags.stream().mapToDouble(Pose2d::getY).average().orElse(0.0);
        return new Pose2d(x, y, Rotation2d.kZero);
    }

    public static Pose2d closestTrenchNeutralSide(AprilTagFieldLayout layout, Alliance alliance, Pose2d robot) {
        Pose2d depotSide = tagPose(layout, trenchNeutralDepotSideId(alliance));
        Pose2d outpostSide = tagPose(layout, trenchNeutralOutpostSideId(alliance));
        double toDepot = robot.getTranslation().getDistance(depotSide.getTranslation());
        double toOutpost = robot.getTranslation().getDistance(outpostSide.getTranslation());
        return toDepot <= toOutpost ? depotSide : outpostSide;
    }

    public static boolean isBetweenTagsX(AprilTagFieldLayout layout, int tagA, int tagB, Pose2d robot) {
        double a = tagPose(layout, tagA).getX();
        double b = tagPose(layout, tagB).getX();
        return robot.getX() >= Math.min(a, b) && robot.getX() <= Math.max(a, b);
    }

    public static boolean isInAllianceZone(AprilTagFieldLayout layout, Alliance alliance, Pose2d robot) {
        return isBetweenTagsX(layout, trenchDriverDepotSideId(alliance), outpostId(alliance), robot);
    }

    public static Translation2d fieldCenter(AprilTagFieldLayout layout) {
        return new Translation2d(layout.getFieldLength() / 2.0, layout.getFieldWidth() / 2.0);
    }

    public static boolean isOnAllianceSide(AprilTagFieldLayout layout, Pose2d pose, Alliance alliance) {
        double centerX = fieldCenter(layout).getX();
        return blue(alliance) ? pose.getX() < centerX : pose.getX() > centerX;
    }

    public static Pose2d closestAutoBallPitEdge(AprilTagFieldLayout layout, Pose2d robot, Alliance alliance) {
        return facingAlongPit(layout, robot.nearest(ballPitEdges(layout, alliance)));
    }

    public static Pose2d furthestAutoBallPitEdge(AprilTagFieldLayout layout, Pose2d robot, Alliance alliance) {
        List<Pose2d> edges = ballPitEdges(layout, alliance);
        Pose2d furthest = edges.get(0);
        double maxDistance = 0.0;
        for (Pose2d edge : edges) {
            double distance = robot.getTranslation().getDistance(edge.getTranslation());
            if (distance > maxDistance) {
                maxDistance = distance;
                furthest = edge;
            }
        }
        return facingAlongPit(layout, furthest);
    }

    public static List<Pose2d> allianceTrenchPoses(AprilTagFieldLayout layout, Alliance alliance) {
        return tagPoses(
                layout,
                blue(alliance)
                        ? List.of(BLUE_TRENCH_DRIVER_DEPOT_SIDE_ID, BLUE_TRENCH_DRIVER_OUTPOST_SIDE_ID)
                        : List.of(RED_TRENCH_DRIVER_DEPOT_SIDE_ID, RED_TRENCH_DRIVER_OUTPOST_SIDE_ID));
    }

    public static Pose2d tagPose(AprilTagFieldLayout layout, int id) {
        return layout.getTagPose(id)
                .orElseThrow(() -> new IllegalArgumentException("AprilTag " + id + " not in the field layout"))
                .toPose2d();
    }

    private static List<Pose2d> tagPoses(AprilTagFieldLayout layout, List<Integer> ids) {
        return ids.stream()
                .map(layout::getTagPose)
                .flatMap(Optional::stream)
                .map(Pose3d::toPose2d)
                .toList();
    }

    private static List<Pose2d> ballPitEdges(AprilTagFieldLayout layout, Alliance alliance) {
        var center = new Pose2d(fieldCenter(layout), Rotation2d.kZero);
        return Arrays.stream(BALL_PIT_CORNERS)
                .map(center::transformBy)
                .filter(edge -> isOnAllianceSide(layout, edge, alliance))
                .toList();
    }

    private static Pose2d facingAlongPit(AprilTagFieldLayout layout, Pose2d edge) {
        return new Pose2d(
                edge.getTranslation(),
                edge.getY() > fieldCenter(layout).getY() ? Rotation2d.kCW_Pi_2 : Rotation2d.kCCW_Pi_2);
    }
}
