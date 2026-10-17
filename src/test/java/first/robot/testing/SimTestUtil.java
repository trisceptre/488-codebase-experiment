package first.robot.testing;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.LoopParticipant;
import java.util.List;

public final class SimTestUtil {
    public static final double LOOP_SECONDS = 0.02;

    private SimTestUtil() {}

    public static void setUp() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        setEnabled(true);
    }

    public static void tearDown() {
        setEnabled(false);
        SimHooks.resumeTiming();
    }

    public static void setEnabled(boolean enabled) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    public static void step(List<? extends LoopParticipant> participants, Runnable betweenPhases) {
        participants.forEach(LoopParticipant::updateInputs);
        betweenPhases.run();
        participants.forEach(LoopParticipant::applyOutputs);
        SimHooks.stepTiming(LOOP_SECONDS);
    }

    public static void schedule(CommandScheduler scheduler, Command command) {
        scheduler.schedule(command);
        assertTrue(scheduler.isScheduled(command), command.getName() + " was not scheduled");
    }

    public static void resetScheduler(CommandScheduler scheduler) {
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
        scheduler.clearComposedCommands();
    }

    public static void step(List<? extends LoopParticipant> participants) {
        step(participants, () -> {});
    }
}
