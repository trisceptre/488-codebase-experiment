package first.robot.subsystems.lights;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import first.robot.testing.SimTestUtil;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LightsTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final List<LightPattern> applied = new ArrayList<>();
    private Lights lights;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        SimTestUtil.resetScheduler(scheduler);
        lights = new Lights(
                new LightsIO() {
                    @Override
                    public void apply(int segment, LightPattern pattern) {
                        applied.add(pattern);
                    }
                },
                3);
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.resetScheduler(scheduler);
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
        DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
        DriverStationSim.notifyNewData();
        loops(2);
        assertEquals(new LightPattern.Larson(Color.kRed), lights.current(2));
    }

    @Test
    void commandOverridesDefaultAndOnlyChangesAreSent() {
        SimTestUtil.schedule(scheduler, lights.solid(Color.kWhite));
        loops(5);
        assertEquals(new LightPattern.Solid(Color.kWhite), lights.current(0));
        assertEquals(
                3,
                applied.stream()
                        .filter(p -> p.equals(new LightPattern.Solid(Color.kWhite)))
                        .count());
    }

    @Test
    void eachSegmentShowsItsOwnPatternAndShortListsRepeatTheLast() {
        var a = new LightPattern.Solid(Color.kGreen);
        var b = new LightPattern.Blink(Color.kRed, 1.0);
        SimTestUtil.schedule(scheduler, lights.show(() -> List.of(a, b)));
        loops(2);
        assertEquals(a, lights.current(0));
        assertEquals(b, lights.current(1));
        assertEquals(b, lights.current(2));
    }
}
