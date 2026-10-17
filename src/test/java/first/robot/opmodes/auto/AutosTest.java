package first.robot.opmodes.auto;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.LoopParticipant;
import first.robot.RobotState;
import first.robot.constants.Hardware2026;
import first.robot.constants.RobotConfigs;
import first.robot.game.GameCommands;
import first.robot.game.GameMechanisms;
import first.robot.game.ShotCalculator;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.FakeDriveIO;
import first.robot.testing.SimTestUtil;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class AutosTest {
    private static final CommandScheduler SCHEDULER = CommandScheduler.getInstance();
    private static RobotState robotState;
    private static GameMechanisms mechanisms;
    private static Autos autos;
    private static List<LoopParticipant> loop;

    @BeforeAll
    static void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(SCHEDULER);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        var field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        var swerve = RobotConfigs.COMP_2026.drive();
        robotState = new RobotState(swerve.kinematics(), swerve.moduleCount());
        var drive = new Drive(swerve, new FakeDriveIO(swerve), robotState);
        mechanisms = GameMechanisms.create(Hardware2026.SUPERSTRUCTURE);
        var game = new GameCommands(mechanisms, drive, robotState, field, new ShotCalculator(field));
        autos = new Autos(drive, robotState, swerve, Optional.of(game));
        var participants = new ArrayList<LoopParticipant>(List.of(drive, robotState));
        participants.addAll(mechanisms.loopParticipants());
        loop = participants;
    }

    @AfterAll
    static void tearDown() {
        DriverStationSim.setAutonomous(false);
        SimTestUtil.resetScheduler(SCHEDULER);
        SimTestUtil.tearDown();
    }

    @Test
    void everyNamedCommandInThePathPlannerFilesIsRegistered() throws IOException {
        Set<String> missing = new TreeSet<>();
        for (String name : namedCommandsInDeployFiles()) {
            if (!NamedCommands.hasCommand(name) && !Autos.UNREGISTERED.contains(name)) {
                missing.add(name);
            }
        }
        assertTrue(missing.isEmpty(), "unregistered named commands: " + missing);
    }

    @Test
    void bumpAutoDrivesBothCyclesAndShoots() {
        var auto = autoNamed("NormalBumpAutoLeft");
        SimTestUtil.schedule(SCHEDULER, auto);
        var start = robotState.getEstimatedPose();
        double farthest = 0.0;
        boolean fired = false;
        boolean extended = false;
        for (int i = 0; i < 20.0 / SimTestUtil.LOOP_SECONDS && SCHEDULER.isScheduled(auto); i++) {
            SimTestUtil.step(loop, SCHEDULER::run);
            farthest = Math.max(
                    farthest, robotState.getEstimatedPose().getTranslation().getDistance(start.getTranslation()));
            fired |= mechanisms.feeder().isFiring();
            extended |= mechanisms.intakeDeploy().goalDeg() < -100.0;
        }
        assertTrue(farthest > 2.0, "drove " + farthest + " m");
        assertTrue(extended, "deployed the intake");
        assertTrue(fired, "fed fuel into the shooter");
        SCHEDULER.cancel(auto);
    }

    private static edu.wpi.first.wpilibj2.command.Command autoNamed(String name) {
        return com.pathplanner.lib.auto.AutoBuilder.buildAuto(name);
    }

    private static Set<String> namedCommandsInDeployFiles() throws IOException {
        var names = new TreeSet<String>();
        var mapper = new ObjectMapper();
        var root = new File(Filesystem.getDeployDirectory(), "pathplanner").toPath();
        try (Stream<Path> files = Files.walk(root)) {
            for (Path file : files.filter(
                            f -> f.toString().endsWith(".auto") || f.toString().endsWith(".path"))
                    .toList()) {
                collect(mapper.readTree(file.toFile()), names);
            }
        }
        return names;
    }

    private static void collect(JsonNode node, Set<String> names) {
        if (node.isObject()) {
            if ("named".equals(node.path("type").asText()) && node.path("data").has("name")) {
                names.add(node.path("data").path("name").asText());
            }
            node.elements().forEachRemaining(child -> collect(child, names));
        } else if (node.isArray()) {
            node.elements().forEachRemaining(child -> collect(child, names));
        }
    }
}
