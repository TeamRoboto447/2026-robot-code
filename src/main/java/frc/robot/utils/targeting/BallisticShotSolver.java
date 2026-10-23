package frc.robot.utils.targeting;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

/**
 * Drag-free projectile model. A hood schedule picks the hood angle for the distance; the solver then
 * finds the launch speed whose arc passes through the target, converts it to flywheel RPM with a
 * linear exit-speed model, and reports the flight time.
 *
 * <p>For a launch pitch {@code a} above horizontal, horizontal distance {@code D} and target height
 * {@code h} above the launch point, the required speed is
 * {@code v^2 = g D^2 / (2 cos^2(a) (D tan(a) - h))} and the flight time is {@code D / (v cos(a))}.
 *
 * <p>Ignoring drag makes the fitted exit-speed coefficients absorb it, so the model is most accurate
 * near the distances it was fit at.
 */
public final class BallisticShotSolver implements ShotSolver {
    /** Gravitational acceleration, m/s^2. */
    public static final double GRAVITY_MPS2 = 9.81;

    /**
     * @param launchHeightM         height of the ball's exit point above the carpet
     * @param exitSpeedInterceptMps exit speed = intercept + perRpm * RPM (before the runtime scale)
     * @param exitSpeedPerRpmMps    exit speed gained per flywheel RPM
     * @param minRpm                slowest usable flywheel speed; slower shots are out of range
     * @param maxRpm                fastest usable flywheel speed; faster shots are out of range
     * @param minHoodDeg            hood travel limits (degrees from vertical); the schedule is clamped to them
     * @param maxHoodDeg            see {@code minHoodDeg}
     */
    public record Params(
        double launchHeightM,
        double exitSpeedInterceptMps,
        double exitSpeedPerRpmMps,
        double minRpm,
        double maxRpm,
        double minHoodDeg,
        double maxHoodDeg) {}

    private final Params params;
    private final DoubleBinaryOperator hoodScheduleDeg;
    private final DoubleSupplier exitSpeedScale;

    /**
     * @param params          model constants
     * @param hoodScheduleDeg hood angle in degrees from vertical, from (distance m, target height above launch m)
     * @param exitSpeedScale  runtime multiplier on the exit-speed model, for range trimming (1.0 = as fit)
     */
    public BallisticShotSolver(Params params, DoubleBinaryOperator hoodScheduleDeg, DoubleSupplier exitSpeedScale) {
        this.params = params;
        this.hoodScheduleDeg = hoodScheduleDeg;
        this.exitSpeedScale = exitSpeedScale;
    }

    /**
     * Hood angle for the lowest-speed arc to the target: launch pitch 45 degrees plus half the elevation
     * angle. Near this angle a small hood error barely moves the landing point.
     *
     * @param distanceM          horizontal distance to the target, meters
     * @param heightAboveLaunchM target height above the launch point, meters
     * @return hood angle in degrees from vertical (not clamped to the hood's travel)
     */
    public static double minimumSpeedHoodDeg(double distanceM, double heightAboveLaunchM) {
        double pitchRad = Math.PI / 4 + 0.5 * Math.atan2(heightAboveLaunchM, distanceM);
        return 90.0 - Math.toDegrees(pitchRad);
    }

    /**
     * @param rpm flywheel speed
     * @return modeled ball exit speed (m/s), including the runtime scale
     */
    public double exitSpeedMps(double rpm) {
        return exitSpeedScale.getAsDouble()
            * (params.exitSpeedInterceptMps() + params.exitSpeedPerRpmMps() * rpm);
    }

    /** @return height of the ball's exit point above the carpet, meters */
    public double launchHeightM() {
        return params.launchHeightM();
    }

    @Override
    public ShotSolution solve(double distanceM, double targetHeightM) {
        double scale = exitSpeedScale.getAsDouble();
        if (!(distanceM > 0) || !(scale > 0)) return null;

        double heightM = targetHeightM - params.launchHeightM();
        double hoodDeg = MathUtil.clamp(
            hoodScheduleDeg.applyAsDouble(distanceM, heightM), params.minHoodDeg(), params.maxHoodDeg());
        double pitchRad = Math.toRadians(90.0 - hoodDeg);
        double cos = Math.cos(pitchRad);
        double rise = distanceM * Math.tan(pitchRad) - heightM;
        // An arc this flat can't climb to the target at any speed.
        if (rise <= 0 || cos < 1e-6) return null;

        double speedMps = Math.sqrt(GRAVITY_MPS2 * distanceM * distanceM / (2 * cos * cos * rise));
        double rpm = (speedMps / scale - params.exitSpeedInterceptMps()) / params.exitSpeedPerRpmMps();
        double timeOfFlightS = distanceM / (speedMps * cos);
        boolean inRange = Double.isFinite(rpm) && rpm >= params.minRpm() && rpm <= params.maxRpm();

        return new ShotSolution(
            hoodDeg,
            MathUtil.clamp(rpm, params.minRpm(), params.maxRpm()),
            timeOfFlightS,
            inRange,
            ShotSolution.Source.PHYSICS);
    }
}
