package frc.team488.mechanisms.lights;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.team488.testing.SimTestUtil;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.util.Color;

class LightsTest {
    private final Scheduler scheduler = Scheduler.getDefault();
    private final List<LightPattern> applied = new ArrayList<>();
    private Lights lights;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        scheduler.cancelAll();
        lights = new Lights(new LightsIO() {
            @Override
            public void apply(LightPattern pattern) {
                applied.add(pattern);
            }
        });
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        lights.close();
        SimTestUtil.tearDown();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            SimTestUtil.step(List.of(lights), scheduler::run);
        }
    }

    @Test
    void defaultShowsAllianceColor() {
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_2);
        DriverStationSim.notifyNewData();
        loops(2);
        assertEquals(new LightPattern.Larson(Color.RED), lights.current());
    }

    @Test
    void commandOverridesDefaultAndOnlyChangesAreSent() {
        SimTestUtil.schedule(scheduler, lights.solid(Color.WHITE));
        loops(5);
        assertEquals(new LightPattern.Solid(Color.WHITE), lights.current());
        assertEquals(
                1,
                applied.stream()
                        .filter(p -> p.equals(new LightPattern.Solid(Color.WHITE)))
                        .count());
    }
}
