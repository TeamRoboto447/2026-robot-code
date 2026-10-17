package frc.robot.utils.targeting;

/**
 * Hood angle, flywheel speed and flight time for a shot from a stationary robot at a given
 * horizontal distance. The {@link ShotPlanner} handles robot motion and aiming; a solution only
 * describes the shot itself.
 *
 * @param hoodDeg       hood angle in degrees from vertical (0 = straight up)
 * @param rpm           flywheel speed
 * @param timeOfFlightS time from leaving the shooter to reaching the target
 * @param inRange       whether the solver trusts this solution (inside its data or limits)
 * @param source        which solver produced it
 */
public record ShotSolution(double hoodDeg, double rpm, double timeOfFlightS, boolean inRange, Source source) {

    /** The solver that produced a solution. */
    public enum Source {
        LUT,
        PHYSICS
    }
}
