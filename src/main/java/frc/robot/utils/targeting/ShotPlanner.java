package frc.robot.utils.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Turns the robot's pose and velocity into a turret angle and a shot. Pure math, so it can be
 * tested on the desktop: no motors, NetworkTables or clocks.
 *
 * <h2>Steps</h2>
 * <ol>
 *   <li>Project the robot pose (position and heading) forward by the latency, to where it will be
 *       when the ball leaves.</li>
 *   <li>Place the turret pivot on the projected pose and measure the displacement to the target.</li>
 *   <li>Scale that displacement by the radial (toward/away) latency term.</li>
 *   <li>Solve for the flight time {@code tau} with {@code tau = TOF(|d - v * tau|)} (Newton's method),
 *       where {@code v} is the pivot's field velocity: the ball inherits it, so aiming at the target
 *       shifted by {@code -v * tau} cancels the drift.</li>
 *   <li>Take the hood/RPM from the solver at that virtual distance, and the turret angle from the
 *       bearing to the virtual target and the projected heading.</li>
 * </ol>
 */
public final class ShotPlanner {
    private static final int MAX_NEWTON_ITERS = 8;
    private static final double CONVERGENCE_TOL_S = 0.001;
    private static final double DERIVATIVE_STEP_M = 0.01;
    /** Closer than this to the target (or virtual target), no shot is attempted. */
    private static final double MIN_DISTANCE_M = 0.03;

    private ShotPlanner() {}

    /**
     * @param pivot            actual (not projected) turret pivot position on the field
     * @param targetDistanceM  horizontal distance from the projected pivot to the real target,
     *                         after the radial latency term; NaN with no target
     * @param virtualDistanceM distance to the motion-compensated target the shot is solved for; NaN if none
     * @param aimBearingRad    field bearing to the virtual target, CCW-positive; NaN if none
     * @param turretAngleDeg   turret setpoint (CW-positive from the back, [-180, 180)) before offset
     *                         and travel wrapping; NaN if none
     * @param solution         the shot, or {@code null} when there is no target or no solution
     */
    public record ShotPlan(
        Translation2d pivot,
        double targetDistanceM,
        double virtualDistanceM,
        double aimBearingRad,
        double turretAngleDeg,
        ShotSolution solution) {

        public boolean hasSolution() {
            return solution != null;
        }
    }

    /**
     * @param pose          current robot pose
     * @param fieldSpeeds   robot velocity, field-relative
     * @param pivotOffset   turret pivot position in the robot frame (meters)
     * @param target        target position on the field, or {@code null} for no target
     * @param solver        shot solver, or {@code null} for none
     * @param latencyS      how far ahead to project the pose (seconds)
     * @param rangeLatencyS extra projection applied only along the line to the target (seconds)
     */
    public static ShotPlan plan(
            Pose2d pose,
            ChassisSpeeds fieldSpeeds,
            Translation2d pivotOffset,
            Translation3d target,
            ShotSolver solver,
            double latencyS,
            double rangeLatencyS) {

        Translation2d pivot = pose.getTranslation().plus(pivotOffset.rotateBy(pose.getRotation()));
        if (target == null || solver == null) {
            return new ShotPlan(pivot, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null);
        }

        // ── 1-2. Latency-projected pose and pivot ─────────────────────────────
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double headingRad = pose.getRotation().getRadians() + omega * latencyS;
        Translation2d offset = pivotOffset.rotateBy(new Rotation2d(headingRad));
        double pivotX = pose.getX() + vx * latencyS + offset.getX();
        double pivotY = pose.getY() + vy * latencyS + offset.getY();

        double dx = target.getX() - pivotX;
        double dy = target.getY() - pivotY;
        double distance = Math.hypot(dx, dy);
        if (distance < MIN_DISTANCE_M) {
            return new ShotPlan(pivot, distance, Double.NaN, Double.NaN, Double.NaN, null);
        }

        // ── 3. Radial latency term ────────────────────────────────────────────
        if (Math.abs(rangeLatencyS) > 1e-9) {
            double radialSpeed = (vx * dx + vy * dy) / distance;
            double scaled = Math.max(0.0, distance + radialSpeed * rangeLatencyS);
            dx *= scaled / distance;
            dy *= scaled / distance;
            distance = scaled;
            if (distance < MIN_DISTANCE_M) {
                return new ShotPlan(pivot, distance, Double.NaN, Double.NaN, Double.NaN, null);
            }
        }

        // ── 4. Flight time for the moving shot ────────────────────────────────
        // The pivot also moves because the robot spins: v_pivot = v + omega x r.
        double pivotVx = vx - omega * offset.getY();
        double pivotVy = vy + omega * offset.getX();
        double targetZ = target.getZ();

        double tau = timeOfFlight(solver, distance, targetZ);
        if (Double.isNaN(tau)) {
            return new ShotPlan(pivot, distance, Double.NaN, Double.NaN, Double.NaN, null);
        }
        for (int i = 0; i < MAX_NEWTON_ITERS; i++) {
            double vdx = dx - pivotVx * tau;
            double vdy = dy - pivotVy * tau;
            double virtualDistance = Math.hypot(vdx, vdy);
            if (virtualDistance < MIN_DISTANCE_M) break;

            double tof = timeOfFlight(solver, virtualDistance, targetZ);
            if (Double.isNaN(tof)) break;
            double error = tau - tof;

            double tofUp = timeOfFlight(solver, virtualDistance + DERIVATIVE_STEP_M, targetZ);
            double tofDown = timeOfFlight(solver, virtualDistance - DERIVATIVE_STEP_M, targetZ);
            double dTofdD = (Double.isNaN(tofUp) || Double.isNaN(tofDown))
                ? 0.0 : (tofUp - tofDown) / (2 * DERIVATIVE_STEP_M);
            double dDdTau = -(vdx * pivotVx + vdy * pivotVy) / virtualDistance;
            double errorSlope = 1.0 - dTofdD * dDdTau;
            if (Math.abs(errorSlope) < 1e-6) break;

            double next = Math.max(0.0, tau - error / errorSlope);
            if (!Double.isFinite(next)) break;
            boolean converged = Math.abs(next - tau) < CONVERGENCE_TOL_S;
            tau = next;
            if (converged) break;
        }

        // ── 5. Shot and aim at the virtual target ─────────────────────────────
        double vdx = dx - pivotVx * tau;
        double vdy = dy - pivotVy * tau;
        double virtualDistance = Math.hypot(vdx, vdy);
        if (virtualDistance < MIN_DISTANCE_M) {
            return new ShotPlan(pivot, distance, virtualDistance, Double.NaN, Double.NaN, null);
        }
        double bearingRad = Math.atan2(vdy, vdx);
        ShotSolution solution = solver.solve(virtualDistance, targetZ);
        double turretAngleDeg = (solution != null)
            ? TurretAngles.turretAngleForBearingDeg(headingRad, bearingRad)
            : Double.NaN;
        return new ShotPlan(pivot, distance, virtualDistance, bearingRad, turretAngleDeg, solution);
    }

    private static double timeOfFlight(ShotSolver solver, double distanceM, double targetZ) {
        if (distanceM < MIN_DISTANCE_M) return Double.NaN;
        ShotSolution s = solver.solve(distanceM, targetZ);
        return (s == null) ? Double.NaN : s.timeOfFlightS();
    }
}
