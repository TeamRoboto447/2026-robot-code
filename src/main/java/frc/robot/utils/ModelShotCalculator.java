package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TurretSubsystemConstants;

/**
 * Lightweight model-based shot calculator used for hybrid targeting.
 *
 * <p>This is intentionally simple and bounded so it can run alongside the
 * LUT/Newton solver while the team calibrates a higher-fidelity model.
 */
public final class ModelShotCalculator {
    private static final int MAX_NEWTON_ITERS = 8;
    private static final double CONVERGENCE_TOL_S = 0.001;
    private static final double MIN_TAU_S = 0.05;
    private static final double MAX_TAU_S = 1.20;

    // Initial model coefficients for hybrid bring-up.
    // These are intentionally simple and should be re-fit from practice data.
    private static final double TOF_A_S = 0.085;
    private static final double TOF_B_S_PER_IN = 0.00190;
    private static final double TOF_C_S_PER_IN2 = 0.0000018;

    private static final double HOOD_A_DEG = 21.0;
    private static final double HOOD_B_DEG_PER_IN = 0.095;
    private static final double HOOD_C_DEG_PER_IN2 = 0.000020;

    private static final double RPM_A = 2450.0;
    private static final double RPM_B_PER_IN = 10.0;
    private static final double RPM_C_PER_IN2 = 0.005;

    private static final double MODEL_MIN_DISTANCE_IN = 45.0;
    private static final double MODEL_MAX_DISTANCE_IN = 260.0;
    private static final double MODEL_MAX_SPEED_IPS = 190.0;

    private ModelShotCalculator() {}

    public static ShooterTable.ShotSolution solve(
            double dxIn,
            double dyIn,
            double robotVxIps,
            double robotVyIps,
            double modelDistanceBiasInches,
            double modelRpmOffset,
            double modelTofScale) {

        double distanceIn = Math.hypot(dxIn, dyIn);
        if (distanceIn < 1.0) return null;

        // Newton-recursed TOF solve:
        // E(tau) = tau - T(D(tau)), where D(tau) = |(dx,dy) - (vx,vy)*tau|
        // and T(D) is a fitted polynomial model (not LUT interpolation).
        double tofScale = MathUtil.clamp(modelTofScale, 0.3, 1.7);
        double tau = MathUtil.clamp(modeledTof(distanceIn) * tofScale, MIN_TAU_S, MAX_TAU_S);
        double tauPrev = tau;
        double tauPrevPrev = tau;
        double contractionRate = Double.NaN;

        for (int i = 0; i < MAX_NEWTON_ITERS; i++) {
            double vdx = dxIn - robotVxIps * tau;
            double vdy = dyIn - robotVyIps * tau;
            double d = Math.hypot(vdx, vdy);
            if (d < 1.0) break;

            double t = MathUtil.clamp(modeledTof(d) * tofScale, MIN_TAU_S, MAX_TAU_S);
            double e = tau - t;

            double dTdD = modeledTofDerivative(d) * tofScale;

            double dDdTau = -(vdx * robotVxIps + vdy * robotVyIps) / d;
            double ePrime = 1.0 - dTdD * dDdTau;
            if (Math.abs(ePrime) < 1e-5) break;

            tauPrevPrev = tauPrev;
            tauPrev = tau;

            double nextTau = tau - e / ePrime;
            if (!Double.isFinite(nextTau)) break;
            tau = MathUtil.clamp(nextTau, MIN_TAU_S, MAX_TAU_S);

            if (i >= 1) {
                double deltaN = Math.abs(tau - tauPrev);
                double deltaNm1 = Math.abs(tauPrev - tauPrevPrev);
                if (deltaNm1 > 1e-9) {
                    contractionRate = deltaN / deltaNm1;
                }
            }

            if (Math.abs(tau - tauPrev) < CONVERGENCE_TOL_S) break;
        }

        double vdx = dxIn - robotVxIps * tau;
        double vdy = dyIn - robotVyIps * tau;
        double virtualDistanceIn = Math.hypot(vdx, vdy);
        double biasedDistanceIn = Math.max(1.0, virtualDistanceIn + modelDistanceBiasInches);
        double aimBearingDeg = Math.toDegrees(Math.atan2(vdy, vdx));

        double hoodDeg = MathUtil.clamp(
                HOOD_A_DEG + HOOD_B_DEG_PER_IN * biasedDistanceIn + HOOD_C_DEG_PER_IN2 * biasedDistanceIn * biasedDistanceIn,
                TurretSubsystemConstants.MIN_HOOD_ANGLE.magnitude(),
                TurretSubsystemConstants.MAX_HOOD_ANGLE.magnitude());

        double rpm = MathUtil.clamp(
                RPM_A + RPM_B_PER_IN * biasedDistanceIn + RPM_C_PER_IN2 * biasedDistanceIn * biasedDistanceIn + modelRpmOffset,
                1800.0,
                6000.0);

        double speedIps = Math.hypot(robotVxIps, robotVyIps);
        boolean inRange = biasedDistanceIn >= MODEL_MIN_DISTANCE_IN
                && biasedDistanceIn <= MODEL_MAX_DISTANCE_IN
                && speedIps <= MODEL_MAX_SPEED_IPS
                && Double.isFinite(tau)
                && Double.isFinite(hoodDeg)
                && Double.isFinite(rpm);

        return new ShooterTable.ShotSolution(
                hoodDeg,
                rpm,
                tau,
                aimBearingDeg,
                contractionRate,
                inRange);
    }

    private static double modeledTof(double distanceIn) {
        return TOF_A_S + TOF_B_S_PER_IN * distanceIn + TOF_C_S_PER_IN2 * distanceIn * distanceIn;
    }

    private static double modeledTofDerivative(double distanceIn) {
        return TOF_B_S_PER_IN + 2.0 * TOF_C_S_PER_IN2 * distanceIn;
    }
}
