package first.robot.mechanisms.lights;

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
        lights = new Lights(new LightsIO() {
            @Override
            public void apply(LightPattern pattern) {
                applied.add(pattern);
            }
        });
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
        assertEquals(new LightPattern.Larson(Color.kRed), lights.current());
    }

    @Test
    void commandOverridesDefaultAndOnlyChangesAreSent() {
        SimTestUtil.schedule(scheduler, lights.solid(Color.kWhite));
        loops(5);
        assertEquals(new LightPattern.Solid(Color.kWhite), lights.current());
        assertEquals(
                1,
                applied.stream()
                        .filter(p -> p.equals(new LightPattern.Solid(Color.kWhite)))
                        .count());
    }
}
