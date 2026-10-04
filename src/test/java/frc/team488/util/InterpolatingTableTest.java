package frc.team488.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.hardware.hal.HAL;

class InterpolatingTableTest {
    private static final Map<String, Double> FALLBACK = Map.of("distance", 3.0, "rpm", 3000.0);

    @TempDir
    Path dir;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize());
    }

    private InterpolatingTable table(String csv) throws IOException {
        Path file = dir.resolve("table.csv");
        Files.writeString(file, csv);
        return new InterpolatingTable("Test", file, "distance", FALLBACK);
    }

    private boolean loads(String csv) throws IOException {
        try (var t = table(csv)) {
            return t.isLoaded();
        }
    }

    @Test
    void exactRowsAndMidpoints() throws IOException {
        try (var t = table("# distance in meters\ndistance, rpm\n1.0, 1000\n3.0, 3000\n2.0, 1500\n")) {
            assertTrue(t.isLoaded());
            assertEquals(1500.0, t.get(2.0, "rpm"));
            assertEquals(1250.0, t.get(1.5, "rpm"), 1e-9);
            assertEquals(2250.0, t.get(2.5, "rpm"), 1e-9);
        }
    }

    @Test
    void clampsOutsideRangeAndNeverReturnsNaN() throws IOException {
        try (var t = table("distance,rpm\n1.0,1000\n3.0,3000\n")) {
            assertEquals(1000.0, t.get(-5.0, "rpm"));
            assertEquals(3000.0, t.get(99.0, "rpm"));
            assertFalse(Double.isNaN(t.get(Double.NaN, "rpm")));
        }
    }

    @Test
    void unknownColumnThrows() throws IOException {
        try (var t = table("distance,rpm\n1.0,1000\n3.0,3000\n")) {
            assertThrows(IllegalArgumentException.class, () -> t.get(2.0, "hood"));
        }
    }

    @Test
    void missingFileUsesFallback() {
        try (var t = new InterpolatingTable("Missing", dir.resolve("nope.csv"), "distance", FALLBACK)) {
            assertFalse(t.isLoaded());
            assertEquals(3000.0, t.get(1.0, "rpm"));
        }
    }

    @Test
    void malformedFilesUseFallback() throws IOException {
        assertFalse(loads("distance,rpm\n1.0,abc\n"));
        assertFalse(loads("distance,rpm\n1.0\n"));
        assertFalse(loads("distance,rpm\n1.0,10\n1.0,20\n"));
        assertFalse(loads("speed,rpm\n1.0,10\n"));
        assertFalse(loads("distance,rpm\n"));
        assertFalse(loads(""));
    }
}
