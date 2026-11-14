package first.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class LibBoundaryTest {
    private static final Path LIB_SOURCES = Path.of("src/main/java/first/lib");

    @Test
    void libNeverDependsOnRobotCode() throws IOException {
        try (Stream<Path> files = Files.walk(LIB_SOURCES)) {
            List<String> offenders = files.filter(path -> path.toString().endsWith(".java"))
                    .filter(LibBoundaryTest::mentionsRobotPackage)
                    .map(Path::toString)
                    .toList();
            assertEquals(List.of(), offenders, "first.lib must not reference first.robot");
        }
    }

    private static boolean mentionsRobotPackage(Path path) {
        try {
            return Files.readAllLines(path).stream()
                    .map(String::strip)
                    .filter(line -> !line.startsWith("//") && !line.startsWith("*") && !line.startsWith("/*"))
                    .anyMatch(line -> line.contains("first.robot"));
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }
}
