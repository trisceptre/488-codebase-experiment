package first.robot.game;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import first.robot.testing.SimTestUtil;
import java.io.File;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double OFFSET = 0.45;

    private final ShotCalculator calculator = new ShotCalculator(
            FIELD,
            List.of(new ShotCalculator.TableEntry(2.0, 0.0, 3000), new ShotCalculator.TableEntry(4.0, 0.4, 4000)),
            List.of(new ShotCalculator.TableEntry(1.0, 0.0, 2000), new ShotCalculator.TableEntry(3.0, 0.0, 3000)));

    @BeforeEach
    void setUp() {
        SimTestUtil.setUp();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        SimTestUtil.tearDown();
    }

    private static Pose2d robotAt(double shooterDistance) {
        var hub = Landmarks.allianceHub(FIELD, Alliance.Blue).getTranslation();
        double centerDistance = shooterDistance - edu.wpi.first.math.util.Units.inchesToMeters(23.5);
        return new Pose2d(hub.plus(new Translation2d(centerDistance, 0.0)), Rotation2d.k180deg);
    }

    @Test
    void interpolatesBetweenTableRows() {
        var shot = calculator.hubShot(robotAt(3.0 - OFFSET), false);
        assertEquals(3500, shot.shooterSpeed().in(RPM), 1.0);
        assertEquals(0.2, shot.hoodRatio(), 1e-3);
        assertEquals(180.0, Math.abs(shot.heading().getDegrees()), 0.5);
    }

    @Test
    void zeroHoodUsesItsOwnTable() {
        var shot = calculator.hubShot(robotAt(2.0 - OFFSET), true);
        assertEquals(2500, shot.shooterSpeed().in(RPM), 1.0);
        assertEquals(0.0, shot.hoodRatio());
    }

    @Test
    void closerThanTheTableUsesTheFirstRowAndFartherHasNoShot() {
        assertEquals(
                3000, calculator.hubShot(robotAt(1.0), false).shooterSpeed().in(RPM), 1.0);
        assertSame(ShotCalculator.Shot.NONE, calculator.hubShot(robotAt(5.0), false));
    }

    @Test
    void presetsHaveTheTunedDefaults() {
        var corner = calculator.preset(ShotCalculator.PresetDistance.CORNER);
        assertEquals(3750, corner.shooterSpeed().in(RPM));
        assertEquals(1.0, corner.hoodRatio());
    }

    @Test
    void deployedTablesLoad() {
        var table = ShotCalculator.loadTable(new File(Filesystem.getDeployDirectory(), "trajectories.json"));
        var zeroHood = ShotCalculator.loadTable(new File(Filesystem.getDeployDirectory(), "trajectories_0_hood.json"));
        assertEquals(178, table.size());
        assertEquals(130, zeroHood.size());
        assertTrue(table.get(0).rpm() > 0);
    }
}
