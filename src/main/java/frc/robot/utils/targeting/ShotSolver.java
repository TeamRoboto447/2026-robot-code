package frc.robot.utils.targeting;

/** Produces a static (robot not moving) shot for a horizontal distance to a target. */
public interface ShotSolver {
    /**
     * @param distanceM     horizontal distance from the turret pivot to the target, meters
     * @param targetHeightM height of the target above the carpet, meters
     * @return the shot, or {@code null} if this solver cannot produce one at all
     */
    ShotSolution solve(double distanceM, double targetHeightM);
}
