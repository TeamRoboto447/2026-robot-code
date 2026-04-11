package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

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

    private ModelShotCalculator() {}

    public static ShooterTable.ShotSolution solve(
            double dxIn,
            double dyIn,
            double robotVxIps,
            double robotVyIps,
            ShooterTable shooterTable) {
        if (shooterTable == null || !shooterTable.isLoaded()) return null;

        double distanceIn = Math.hypot(dxIn, dyIn);
        if (distanceIn < 1.0) return null;

        // Newton-recursed TOF solve:
        // E(tau) = tau - T(D(tau)), where D(tau) = |(dx,dy) - (vx,vy)*tau|
        // and T(D) is the LUT interpolated flight time at distance D.
        double tau = MathUtil.clamp(shooterTable.interpolateTime(distanceIn), MIN_TAU_S, MAX_TAU_S);
        double tauPrev = tau;
        double tauPrevPrev = tau;
        double contractionRate = Double.NaN;

        for (int i = 0; i < MAX_NEWTON_ITERS; i++) {
            double vdx = dxIn - robotVxIps * tau;
            double vdy = dyIn - robotVyIps * tau;
            double d = Math.hypot(vdx, vdy);
            if (d < 1.0) break;

            double t = MathUtil.clamp(shooterTable.interpolateTime(d), MIN_TAU_S, MAX_TAU_S);
            double e = tau - t;

            double h = Math.max(1.0, 0.01 * d);
            double tPlus = MathUtil.clamp(shooterTable.interpolateTime(d + h), MIN_TAU_S, MAX_TAU_S);
            double tMinus = MathUtil.clamp(shooterTable.interpolateTime(Math.max(1.0, d - h)), MIN_TAU_S, MAX_TAU_S);
            double dTdD = (tPlus - tMinus) / (2.0 * h);

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
        double aimBearingDeg = Math.toDegrees(Math.atan2(vdy, vdx));

        double hoodDeg = shooterTable.interpolateAngle(virtualDistanceIn);
        double rpm = shooterTable.interpolateRPM(virtualDistanceIn);

        double speedIps = Math.hypot(robotVxIps, robotVyIps);
        // boolean inRange = shooterTable != null
        //         && shooterTable.isLoaded()
        //         && shooterTable.isDistanceInRange(virtualDistanceIn)
        //         && speedIps <= 140.0;
        boolean inRange = true;

        return new ShooterTable.ShotSolution(
                hoodDeg,
                rpm,
                tau,
                aimBearingDeg,
                contractionRate,
                inRange);
    }
}
