package frc.robot.utils.targeting;

/**
 * Uses the primary solver whenever it has an in-range solution, otherwise the fallback's in-range
 * solution. If neither is in range, returns the primary's (out-of-range) solution if it has one.
 */
public final class FallbackShotSolver implements ShotSolver {
    private final ShotSolver primary;
    private final ShotSolver fallback;

    public FallbackShotSolver(ShotSolver primary, ShotSolver fallback) {
        this.primary = primary;
        this.fallback = fallback;
    }

    @Override
    public ShotSolution solve(double distanceM, double targetHeightM) {
        ShotSolution first = primary.solve(distanceM, targetHeightM);
        if (first != null && first.inRange()) return first;
        ShotSolution second = fallback.solve(distanceM, targetHeightM);
        if (second != null && second.inRange()) return second;
        return (first != null) ? first : second;
    }
}
