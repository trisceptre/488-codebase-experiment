package first.robot.game;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import first.lib.LoopParticipant;
import first.lib.RobotState;
import first.lib.drive.Drive;
import first.lib.drive.DriveIO;
import first.lib.testing.SimTestUtil;
import first.robot.constants.Hardware2026;
import first.robot.constants.RobotConfigs;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class GameCommandsTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private GameCommands game;
    private List<LoopParticipant> loop;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        var field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        var driveConfig = RobotConfigs.COMP_2026.drive();
        var robotState = new RobotState(driveConfig.kinematics(), driveConfig.moduleCount());
        var drive = new Drive(driveConfig, new DriveIO() {}, robotState);
        var mechanisms = GameMechanisms.create(Hardware2026.SUPERSTRUCTURE);
        game = new GameCommands(mechanisms, drive, robotState, field, new ShotCalculator(field));
        loop = mechanisms.loopParticipants();
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
        SimTestUtil.tearDown();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            SimTestUtil.step(loop, scheduler::run);
        }
    }

    @Test
    void firesOnlyOnceThePresetIsReady() {
        SimTestUtil.schedule(
                scheduler,
                Commands.parallel(game.prepareToShoot(ShotCalculator.PresetDistance.NEAR), game.fireWhenReady()));
        loops(5);
        assertFalse(game.mechanisms().feeder().isFiring(), "shooter still spinning up");
        loops(150);
        assertTrue(game.mechanisms().shooter().isReadyToFire());
        assertTrue(game.mechanisms().feeder().isFiring());
    }

    @Test
    void collectRunsCollectorAndHopperTogether() {
        SimTestUtil.schedule(scheduler, game.collect());
        loops(2);
        assertTrue(game.mechanisms().collector().isIntaking());
        assertFalse(game.mechanisms().feeder().isFiring());
    }
}
