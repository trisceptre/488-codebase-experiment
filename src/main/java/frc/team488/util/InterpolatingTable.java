package frc.team488.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Filesystem;
import org.wpilib.util.Alert;

/**
 * A lookup table loaded once from a CSV file: a header row of column names, then numeric rows.
 * Blank lines and lines starting with {@code #} are ignored. Values are linearly interpolated on a
 * key column and clamped at the ends. If the file is missing or malformed, an alert is raised and
 * every query returns the fallback row.
 *
 * <p>The alert ID derives from the table name, so two live tables can't share a name. {@link
 * #close()} releases it.
 */
public final class InterpolatingTable implements AutoCloseable {
    private final String name;
    private final Map<String, Double> fallback;
    private final List<String> columns;
    private final double[][] rows;
    private final int keyIndex;
    private final boolean loaded;
    private final Alert loadFailedAlert;

    /** Loads {@code fileName} from the deploy directory. */
    public static InterpolatingTable fromDeploy(
            String name, String fileName, String keyColumn, Map<String, Double> fallback) {
        return new InterpolatingTable(
                name, Filesystem.getDeployDirectory().toPath().resolve(fileName), keyColumn, fallback);
    }

    public InterpolatingTable(String name, Path file, String keyColumn, Map<String, Double> fallback) {
        this.name = name;
        this.fallback = Map.copyOf(fallback);
        Parsed parsed = null;
        String failure = null;
        try {
            parsed = Parsed.read(file, keyColumn);
        } catch (IOException e) {
            failure = "cannot read " + file + ": " + e.getMessage();
        } catch (IllegalArgumentException e) {
            failure = e.getMessage();
        }
        loaded = parsed != null;
        columns = loaded ? parsed.columns() : List.copyOf(this.fallback.keySet());
        rows = loaded ? parsed.rows() : new double[0][];
        keyIndex = loaded ? parsed.keyIndex() : -1;
        loadFailedAlert = new Alert(
                name + "/LoadFailed",
                "Table " + name + " failed to load (" + failure + "); using fallback values",
                Alert.Level.HIGH);
        loadFailedAlert.set(!loaded);
        if (!loaded) {
            System.err.println(loadFailedAlert.getText());
        }
    }

    /** Releases the load-failure alert. */
    @Override
    public void close() {
        loadFailedAlert.close();
    }

    public boolean isLoaded() {
        return loaded;
    }

    /** The value of {@code column} at {@code key}; clamped to the table's ends, never NaN. */
    public double get(double key, String column) {
        if (!loaded) {
            Double value = fallback.get(column);
            if (value == null) {
                throw new IllegalArgumentException("Table " + name + " fallback has no column '" + column + "'");
            }
            return value;
        }
        int col = columns.indexOf(column);
        if (col < 0) {
            throw new IllegalArgumentException(
                    "Table " + name + " has no column '" + column + "'; columns are " + columns);
        }
        double[] first = rows[0];
        double[] last = rows[rows.length - 1];
        boolean outOfRange = Double.isNaN(key) || key < first[keyIndex] || key > last[keyIndex];
        Logger.recordOutput(name + "/OutOfRange", outOfRange);
        if (Double.isNaN(key) || key <= first[keyIndex]) {
            return first[col];
        }
        if (key >= last[keyIndex]) {
            return last[col];
        }
        int upper = 1;
        while (rows[upper][keyIndex] < key) {
            upper++;
        }
        double[] a = rows[upper - 1];
        double[] b = rows[upper];
        double t = (key - a[keyIndex]) / (b[keyIndex] - a[keyIndex]);
        return a[col] + t * (b[col] - a[col]);
    }

    private record Parsed(List<String> columns, double[][] rows, int keyIndex) {
        static Parsed read(Path file, String keyColumn) throws IOException {
            List<String> lines = Files.readAllLines(file).stream()
                    .map(String::strip)
                    .filter(line -> !line.isEmpty() && !line.startsWith("#"))
                    .toList();
            if (lines.isEmpty()) {
                throw new IllegalArgumentException(file + " is empty");
            }
            List<String> columns = Arrays.stream(lines.getFirst().split(","))
                    .map(String::strip)
                    .toList();
            int keyIndex = columns.indexOf(keyColumn);
            if (keyIndex < 0) {
                throw new IllegalArgumentException(file + " has no key column '" + keyColumn + "'");
            }
            if (lines.size() < 2) {
                throw new IllegalArgumentException(file + " has no data rows");
            }
            double[][] rows = new double[lines.size() - 1][];
            for (int r = 0; r < rows.length; r++) {
                String[] cells = lines.get(r + 1).split(",", -1);
                if (cells.length != columns.size()) {
                    throw new IllegalArgumentException(file + " data row " + (r + 1) + " has " + cells.length
                            + " values; expected " + columns.size());
                }
                rows[r] = new double[cells.length];
                for (int c = 0; c < cells.length; c++) {
                    rows[r][c] = parseFinite(cells[c].strip(), file, r + 1);
                }
            }
            Arrays.sort(rows, Comparator.comparingDouble(row -> row[keyIndex]));
            for (int r = 1; r < rows.length; r++) {
                if (rows[r][keyIndex] == rows[r - 1][keyIndex]) {
                    throw new IllegalArgumentException(file + " has duplicate key " + rows[r][keyIndex]);
                }
            }
            return new Parsed(columns, rows, keyIndex);
        }

        private static double parseFinite(String cell, Path file, int row) {
            double value;
            try {
                value = Double.parseDouble(cell);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException(file + " data row " + row + ": '" + cell + "' is not a number");
            }
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(file + " data row " + row + ": '" + cell + "' is not finite");
            }
            return value;
        }
    }
}
