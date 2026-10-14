package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.targeting.ShooterTable.DataPoint;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShooterTableTest {
    private static final double EPS = 1e-9;

    private static ShooterTable table(double marginIn) {
        // Deliberately unsorted.
        return new ShooterTable(List.of(
            new DataPoint(200.0, 40.0, 5000, 1.0),
            new DataPoint(100.0, 30.0, 4000, 0.6)), marginIn);
    }

    @Test
    void interpolatesBetweenPoints() {
        ShooterTable t = table(0);
        assertEquals(35.0, t.interpolateAngle(150.0), EPS);
        assertEquals(4500.0, t.interpolateRPM(150.0), EPS);
        assertEquals(0.8, t.interpolateTime(150.0), EPS);
    }

    @Test
    void clampsBeyondEnds() {
        ShooterTable t = table(0);
        assertEquals(30.0, t.interpolateAngle(10.0), EPS);
        assertEquals(5000.0, t.interpolateRPM(400.0), EPS);
    }

    @Test
    void rangeIncludesMargin() {
        ShooterTable t = table(24.0);
        assertTrue(t.isDistanceInRange(76.0));
        assertTrue(t.isDistanceInRange(224.0));
        assertFalse(t.isDistanceInRange(75.9));
        assertFalse(t.isDistanceInRange(224.1));
    }

    @Test
    void solveTakesMetersAndFlagsRange() {
        ShooterTable t = table(24.0);
        ShotSolution s = t.solve(Units.inchesToMeters(150.0), 1.83);
        assertEquals(35.0, s.hoodDeg(), 1e-6);
        assertEquals(4500.0, s.rpm(), 1e-6);
        assertTrue(s.inRange());
        assertEquals(ShotSolution.Source.LUT, s.source());

        ShotSolution far = t.solve(Units.inchesToMeters(300.0), 1.83);
        assertFalse(far.inRange());
        assertEquals(5000.0, far.rpm(), EPS);
    }

    @Test
    void emptyTableGivesNoSolution() {
        ShooterTable t = new ShooterTable(List.of(), 24.0);
        assertFalse(t.isLoaded());
        assertNull(t.solve(3.0, 1.83));
    }

    @Test
    void dropsInvalidAndDuplicatePoints() {
        ShooterTable t = new ShooterTable(List.of(
            new DataPoint(100.0, 30.0, 4000, 0.6),
            new DataPoint(100.0, 31.0, 4100, 0.6),
            new DataPoint(150.0, Double.NaN, 4100, 0.6),
            new DataPoint(160.0, 35.0, 4500, 0.0),
            new DataPoint(200.0, 40.0, 5000, 1.0)), 0);
        assertEquals(2, t.size());
        assertEquals(30.0, t.interpolateAngle(100.0), EPS);
    }

    @Test
    void parsesGeneratorFormat() {
        String json = "{\n  \"data_points\": [\n"
            + "    {\"target_height\": 72.0, \"distance\": 73.7, \"vpar\": 0.0, \"angle\": 30.18, \"rpm\": 3625, \"flight_time\": 0.98},\n"
            + "    {\"distance\": 111.0, \"angle\": 36.16, \"rpm\": 3725, \"flight_time\": 1.5e-1},\n"
            + "    {\"distance\": 120.0, \"angle\": 36.0}\n"
            + "  ]\n}";
        List<DataPoint> points = ShooterTable.parse(json);
        assertEquals(2, points.size());
        assertEquals(new DataPoint(73.7, 30.18, 3625, 0.98), points.get(0));
        assertEquals(0.15, points.get(1).timeS(), EPS);
    }
}
