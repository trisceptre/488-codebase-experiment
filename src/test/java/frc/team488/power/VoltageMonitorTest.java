package frc.team488.power;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team488.testing.SimTestUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VoltageMonitorTest {
    private double volts = 12.5;
    private VoltageMonitor monitor;

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        monitor = new VoltageMonitor(new PowerIO() {
            @Override
            public void updateInputs(PowerIOInputs inputs) {
                inputs.connected = true;
                inputs.voltage = volts;
            }
        });
    }

    @AfterEach
    void tearDown() {
        monitor.close();
        SimTestUtil.tearDown();
    }

    @Test
    void lowVoltageIsDebounced() {
        volts = 7.5;
        for (int i = 0; i < 25; i++) {
            SimTestUtil.step(List.of(monitor));
        }
        assertFalse(monitor.isBrownoutRisk(), "before 1 s");
        for (int i = 0; i < 30; i++) {
            SimTestUtil.step(List.of(monitor));
        }
        assertTrue(monitor.isBrownoutRisk());
        volts = 12.0;
        SimTestUtil.step(List.of(monitor));
        assertFalse(monitor.isBrownoutRisk());
    }
}
