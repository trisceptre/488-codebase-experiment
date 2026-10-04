package frc.team488.testing;

import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.LoopParticipant;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/** Deterministic simulated robot loops for tests. */
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

    /** One robot loop in production order: inputs, then {@code betweenPhases}, then outputs. */
    public static void step(List<? extends LoopParticipant> participants, Runnable betweenPhases) {
        participants.forEach(LoopParticipant::updateInputs);
        betweenPhases.run();
        participants.forEach(LoopParticipant::applyOutputs);
        SimHooks.stepTiming(LOOP_SECONDS);
    }

    /** Schedules {@code command} and fails the test if the scheduler rejects it. */
    public static void schedule(Scheduler scheduler, Command command) {
        assertInstanceOf(Scheduler.ScheduleResult.Successful.class, scheduler.schedule(command));
    }

    public static void step(List<? extends LoopParticipant> participants) {
        step(participants, () -> {});
    }
}
