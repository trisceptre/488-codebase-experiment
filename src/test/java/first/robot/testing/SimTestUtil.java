package first.robot.testing;

import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.LoopParticipant;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

public final class SimTestUtil {
    public static final double LOOP_SECONDS = 0.02;

    private SimTestUtil() {}

    public static void setUp() {
        assertTrue(HAL.initialize());
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

    public static void schedule(Scheduler scheduler, Command command) {
        assertInstanceOf(Scheduler.ScheduleResult.Successful.class, scheduler.schedule(command));
    }

    public static void step(List<? extends LoopParticipant> participants) {
        step(participants, () -> {});
    }
}
