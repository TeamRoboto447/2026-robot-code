package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.robot.Constants.TurretSubsystemConstants;
import org.junit.jupiter.api.Test;

class TurretAngleTrimsTest {
    private static final double EPS = 1e-9;

    // Index i holds the value at turret angle -180 + 45 * i: -180, -135, -90, -45, 0, 45, 90, 135.
    private static final double[] DISTANCE = {80, 70, 60, 50, 40, 30, 20, 10};
    private static final double[] YAW = {-4, -3, -2, -1, 0, 1, 2, 3};

    @Test
    void tablePointsAreEvery45DegreesFromTheFront() {
        assertEquals(8, TurretAngleTrims.POINTS);
        assertEquals(-180.0, TurretAngleTrims.angleOfPoint(0), EPS);
        assertEquals(0.0, TurretAngleTrims.angleOfPoint(4), EPS);
        assertEquals(135.0, TurretAngleTrims.angleOfPoint(7), EPS);
    }

    @Test
    void returnsTableValuesExactlyAtPoints() {
        for (int i = 0; i < TurretAngleTrims.POINTS; i++) {
            var trim = TurretAngleTrims.at(TurretAngleTrims.angleOfPoint(i), DISTANCE, YAW);
            assertEquals(DISTANCE[i], trim.distanceIn(), EPS);
            assertEquals(YAW[i], trim.yawDeg(), EPS);
        }
    }

    @Test
    void interpolatesBetweenPoints() {
        var trim = TurretAngleTrims.at(-67.5, DISTANCE, YAW); // halfway between -90 and -45
        assertEquals(55.0, trim.distanceIn(), EPS);
        assertEquals(-1.5, trim.yawDeg(), EPS);

        trim = TurretAngleTrims.at(9.0, DISTANCE, YAW); // 20% of the way from 0 to 45
        assertEquals(38.0, trim.distanceIn(), EPS);
        assertEquals(0.2, trim.yawDeg(), EPS);
    }

    @Test
    void wrapsBetween135AndTheFront() {
        // 157.5 is halfway from 135 (index 7) back around to -180/+180 (index 0).
        var trim = TurretAngleTrims.at(157.5, DISTANCE, YAW);
        assertEquals(45.0, trim.distanceIn(), EPS);
        assertEquals(-0.5, trim.yawDeg(), EPS);

        // +180 is the same direction as -180.
        assertEquals(80.0, TurretAngleTrims.at(180.0, DISTANCE, YAW).distanceIn(), EPS);
    }

    @Test
    void wrapsAnglesOutsideOneTurn() {
        // The turret travel runs past +/-180 (e.g. -190), and a setpoint may be a full turn off.
        assertEquals(
            TurretAngleTrims.at(170.0, DISTANCE, YAW).distanceIn(),
            TurretAngleTrims.at(-190.0, DISTANCE, YAW).distanceIn(), EPS);
        assertEquals(
            TurretAngleTrims.at(-90.0, DISTANCE, YAW).distanceIn(),
            TurretAngleTrims.at(270.0, DISTANCE, YAW).distanceIn(), EPS);
    }

    @Test
    void nonFiniteAngleGivesNoTrim() {
        assertEquals(TurretAngleTrims.Trim.NONE, TurretAngleTrims.at(Double.NaN, DISTANCE, YAW));
    }

    @Test
    void rejectsWrongSizedTables() {
        assertThrows(IllegalArgumentException.class,
            () -> TurretAngleTrims.at(0.0, new double[] {1, 2, 3}, YAW));
    }

    @Test
    void tunedConstantsFitTheTable() {
        double[] distance = TurretSubsystemConstants.TURRET_ANGLE_DISTANCE_TRIM_IN;
        double[] yaw = TurretSubsystemConstants.TURRET_ANGLE_YAW_TRIM_DEG;
        assertEquals(TurretAngleTrims.POINTS, distance.length);
        assertEquals(TurretAngleTrims.POINTS, yaw.length);
        // The shooter is tuned with the turret at 0, so that point needs no trim.
        assertEquals(TurretAngleTrims.Trim.NONE, TurretAngleTrims.at(0.0, distance, yaw));
    }
}
