package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;

class FallbackShotSolverTest {
    private static final ShotSolution PHYSICS_IN = new ShotSolution(30, 4000, 0.7, true, ShotSolution.Source.PHYSICS);
    private static final ShotSolution PHYSICS_OUT = new ShotSolution(30, 6000, 0.7, false, ShotSolution.Source.PHYSICS);
    private static final ShotSolution LUT_IN = new ShotSolution(31, 4100, 0.9, true, ShotSolution.Source.LUT);
    private static final ShotSolution LUT_OUT = new ShotSolution(31, 4100, 0.9, false, ShotSolution.Source.LUT);

    private static ShotSolver returning(ShotSolution s) {
        return (d, h) -> s;
    }

    @Test
    void prefersPrimaryWhenInRange() {
        assertSame(PHYSICS_IN, new FallbackShotSolver(returning(PHYSICS_IN), returning(LUT_IN)).solve(3, 1.8));
    }

    @Test
    void usesFallbackWhenPrimaryOutOfRangeOrMissing() {
        assertSame(LUT_IN, new FallbackShotSolver(returning(PHYSICS_OUT), returning(LUT_IN)).solve(3, 1.8));
        assertSame(LUT_IN, new FallbackShotSolver(returning(null), returning(LUT_IN)).solve(3, 1.8));
    }

    @Test
    void neitherInRangeReturnsPrimaryThenFallback() {
        assertSame(PHYSICS_OUT, new FallbackShotSolver(returning(PHYSICS_OUT), returning(LUT_OUT)).solve(3, 1.8));
        assertSame(LUT_OUT, new FallbackShotSolver(returning(null), returning(LUT_OUT)).solve(3, 1.8));
        assertNull(new FallbackShotSolver(returning(null), returning(null)).solve(3, 1.8));
    }
}
