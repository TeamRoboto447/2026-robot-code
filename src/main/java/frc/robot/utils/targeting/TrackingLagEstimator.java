package frc.robot.utils.targeting;

import edu.wpi.first.math.MathUtil;

/**
 * Measures how far a position loop lags behind a moving setpoint, in seconds.
 *
 * <p>When a servo follows a setpoint moving at a steady rate, it settles into following it a fixed
 * time behind, so {@code lag = tracking error / setpoint rate}. Samples are only taken during steady
 * motion: not while standing still, not on jumps (target switches, wrapping across the turret seam),
 * and only once the motion has lasted a few lag-times so the error has settled. Accepted samples are
 * low-pass filtered.
 *
 * <p>The measured position is older than the setpoint it is compared with (CAN status frames). While
 * moving, that age alone looks like {@code rate * age} of tracking error, so it is subtracted.
 */
public final class TrackingLagEstimator {
    private final double minRateDegPerS;
    private final double maxRateDegPerS;
    private final double maxErrorDeg;
    private final double filterTimeConstantS;
    private final double maxLagS;

    private double lagS;
    private double prevSetpointDeg = Double.NaN;
    private double prevRateDegPerS = Double.NaN;
    private double steadyMotionS = 0.0;

    /**
     * @param initialLagS         estimate to use until samples arrive
     * @param minRateDegPerS      slower setpoint motion is ignored (error is mostly noise and friction)
     * @param maxRateDegPerS      faster setpoint motion is treated as a jump, not tracking
     * @param maxErrorDeg         larger tracking errors are ignored (the loop is catching up, not tracking)
     * @param filterTimeConstantS low-pass time constant applied to accepted samples
     * @param maxLagS             the estimate is clamped to [0, maxLagS]
     */
    public TrackingLagEstimator(
            double initialLagS,
            double minRateDegPerS,
            double maxRateDegPerS,
            double maxErrorDeg,
            double filterTimeConstantS,
            double maxLagS) {
        this.lagS = MathUtil.clamp(initialLagS, 0.0, maxLagS);
        this.minRateDegPerS = minRateDegPerS;
        this.maxRateDegPerS = maxRateDegPerS;
        this.maxErrorDeg = maxErrorDeg;
        this.filterTimeConstantS = filterTimeConstantS;
        this.maxLagS = maxLagS;
    }

    /** @return the current lag estimate, seconds */
    public double getLagSeconds() {
        return lagS;
    }

    /**
     * Feeds one control loop.
     *
     * @param tracking        whether the loop commanded the setpoint this loop (false resets the motion tracking)
     * @param setpointDeg     setpoint commanded this loop
     * @param measuredDeg     latest measured position
     * @param measurementAgeS how old {@code measuredDeg} is, seconds
     * @param dtS             time since the previous call, seconds
     */
    public void update(boolean tracking, double setpointDeg, double measuredDeg, double measurementAgeS, double dtS) {
        if (!tracking || !(dtS > 0) || !Double.isFinite(setpointDeg) || !Double.isFinite(measuredDeg)) {
            prevSetpointDeg = tracking ? setpointDeg : Double.NaN;
            prevRateDegPerS = Double.NaN;
            steadyMotionS = 0.0;
            return;
        }
        if (Double.isNaN(prevSetpointDeg)) {
            prevSetpointDeg = setpointDeg;
            return;
        }

        double rate = (setpointDeg - prevSetpointDeg) / dtS;
        boolean sameDirection = Double.isNaN(prevRateDegPerS) || Math.signum(rate) == Math.signum(prevRateDegPerS);
        prevSetpointDeg = setpointDeg;
        prevRateDegPerS = rate;

        double absRate = Math.abs(rate);
        if (absRate < minRateDegPerS || absRate > maxRateDegPerS || !sameDirection) {
            steadyMotionS = 0.0;
            return;
        }
        steadyMotionS += dtS;
        // A first-order lag reaches 95% of its steady error after 3 time constants.
        if (steadyMotionS < 3.0 * lagS + dtS) return;

        double error = setpointDeg - measuredDeg;
        if (Math.abs(error) > maxErrorDeg || Math.signum(error) != Math.signum(rate)) return;

        double sampleS = error / rate - Math.max(0.0, measurementAgeS);
        double alpha = dtS / (filterTimeConstantS + dtS);
        lagS = MathUtil.clamp(lagS + alpha * (sampleS - lagS), 0.0, maxLagS);
    }
}
