package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.TurretSubsystemConstants;
import org.junit.jupiter.api.Test;

class BallisticShotSolverTest {
    private static final double HUB_HEIGHT_M = 1.83;

    private static BallisticShotSolver.Params params() {
        return new BallisticShotSolver.Params(
            TurretSubsystemConstants.PHYSICS_LAUNCH_HEIGHT_M,
            TurretSubsystemConstants.PHYSICS_EXIT_SPEED_INTERCEPT_MPS,
            TurretSubsystemConstants.PHYSICS_EXIT_SPEED_PER_RPM_MPS,
            TurretSubsystemConstants.PHYSICS_MIN_RPM,
            TurretSubsystemConstants.PHYSICS_MAX_RPM,
            TurretSubsystemConstants.MIN_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees),
            TurretSubsystemConstants.MAX_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees));
    }

    private static BallisticShotSolver fixedHood(double hoodDeg, double scale) {
        return new BallisticShotSolver(params(), (d, h) -> hoodDeg, () -> scale);
    }

    /** Height of the modeled arc at the target's horizontal distance. */
    private static double arcHeightAt(BallisticShotSolver solver, ShotSolution s, double distanceM) {
        double v = solver.exitSpeedMps(s.rpm());
        double pitch = Math.toRadians(90.0 - s.hoodDeg());
        double t = distanceM / (v * Math.cos(pitch));
        return solver.launchHeightM() + v * Math.sin(pitch) * t - 0.5 * BallisticShotSolver.GRAVITY_MPS2 * t * t;
    }

    @Test
    void arcPassesThroughTarget() {
        BallisticShotSolver solver = fixedHood(36.0, 1.0);
        for (double d = 2.0; d <= 5.5; d += 0.5) {
            ShotSolution s = solver.solve(d, HUB_HEIGHT_M);
            assertNotNull(s);
            assertTrue(s.inRange());
            assertEquals(HUB_HEIGHT_M, arcHeightAt(solver, s, d), 1e-6, "at " + d + " m");
            double v = solver.exitSpeedMps(s.rpm());
            assertEquals(d / (v * Math.cos(Math.toRadians(54.0))), s.timeOfFlightS(), 1e-9);
        }
    }

    @Test
    void higherExitSpeedScaleNeedsLessRpm() {
        double base = fixedHood(36.0, 1.0).solve(3.0, HUB_HEIGHT_M).rpm();
        double scaled = fixedHood(36.0, 1.1).solve(3.0, HUB_HEIGHT_M).rpm();
        assertTrue(scaled < base);
    }

    @Test
    void tooFlatAnArcHasNoSolution() {
        // A flat launch can't climb 1.3 m in 0.5 m of distance.
        assertNull(fixedHood(43.0, 1.0).solve(0.5, HUB_HEIGHT_M));
    }

    @Test
    void outOfRpmBandIsOutOfRange() {
        ShotSolution s = fixedHood(43.0, 1.0).solve(25.0, HUB_HEIGHT_M);
        assertNotNull(s);
        assertFalse(s.inRange());
        assertEquals(TurretSubsystemConstants.PHYSICS_MAX_RPM, s.rpm(), 1e-9);
    }

    @Test
    void hoodScheduleIsClampedToTravel() {
        ShotSolution s = fixedHood(80.0, 1.0).solve(3.0, 0.0);
        assertEquals(TurretSubsystemConstants.MAX_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees), s.hoodDeg(), 1e-9);
    }

    @Test
    void minimumSpeedHoodIs45DegreesForLevelTarget() {
        assertEquals(45.0, BallisticShotSolver.minimumSpeedHoodDeg(4.0, 0.0), 1e-9);
        // A raised target needs a steeper (smaller from-vertical) hood.
        assertTrue(BallisticShotSolver.minimumSpeedHoodDeg(4.0, 1.3) < 45.0);
    }
}
