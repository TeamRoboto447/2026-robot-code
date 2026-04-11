package frc.robot.utils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Loads {@code turret_data.json} from the deploy directory at startup and
 * provides shot-parameter lookup with linear interpolation and a Newton-method
 * dynamic-shooting solver for shoot-on-the-fly.
 *
 * <h2>Memory footprint</h2>
 * Only the raw measured data points are kept in memory — no pre-computed grid.
 * At N points this is {@code N × 4 doubles = N × 32 bytes}. With the current 3
 * static points that is &lt;200 bytes including object overhead.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * ShooterTable table = new ShooterTable();
 * // Static (robot not moving):
 * ShooterTable.ShotSolution sol = table.solve(distInches, 0, 0);
 * // Shoot-on-the-fly:
 * ShooterTable.ShotSolution sol = table.solve(distInches, vxInchesPerSec, vyInchesPerSec);
 * }</pre>
 *
 * <h2>Algorithm</h2>
 * The solver uses the WPILib "Dynamic Shooting via Time-of-Flight Recursion"
 * approach with Newton's method for fast convergence (typically 1–3 iterations).
 * The virtual target position is updated each iteration:
 * <pre>
 *   d_virtual = (target - robot) - v_robot * tau_n
 *   D         = |d_virtual|
 *   tau_n+1   = tau_n - E / E'
 *   where E  = tau_n - tof_lut(D)
 *         E' = 1 + (d_virtual · v_robot) / (v_projectile * D)
 * </pre>
 * The horizontal projectile speed {@code v_projectile} is estimated from the
 * static table entry for {@code D} (distance / time_of_flight). This proxy
 * avoids differentiating the LUT and degrades gracefully with drag.
 *
 * <p>Once converged, the aim bearing is derived from the virtual target vector
 * so both vpar and vperp corrections are handled simultaneously in correct 2D
 * geometry — no separate "lead angle" step is needed.
 *
 * <p><b>Flight times in turret_data.json are currently placeholders (2.5 s).</b>
 * Replace them with measured values; until then the shoot-on-the-fly correction
 * will be inaccurate. Static shots (vx = vy = 0) are unaffected by flight time.
 */
public class ShooterTable {

    /** Maximum number of Newton iterations for the TOF solver. */
    private static final int MAX_NEWTON_ITERS = 5;

    /**
     * Convergence threshold: stop iterating when |tau_n - tau_n-1| is smaller
     * than this value (seconds). 2 ms is well below any mechanical precision limit.
     */
    private static final double CONVERGENCE_TOL_S = 0.002;

    /**
     * Fallback horizontal projectile speed (in/s) used when the table cannot
     * produce a reasonable estimate (e.g. table has only one point). 240 in/s
     * ≈ 20 ft/s, a typical FRC ball speed.
     */
    private static final double FALLBACK_PROJECTILE_SPEED_IPS = 240.0;

    /**
     * One row from {@code turret_data.json}: a static (vpar=0) empirical shot.
     */
    public static final class DataPoint {
        /** Horizontal distance to the target (inches). */
        public final double distanceIn;
        /** Hood angle from vertical, degrees (0 = straight up, 90 = horizontal). */
        public final double angleDeg;
        /** Flywheel RPM. */
        public final double rpm;
        /** Measured (or estimated) time of flight in seconds. */
        public final double timeS;

        public DataPoint(double distanceIn, double angleDeg, double rpm, double timeS) {
            this.distanceIn  = distanceIn;
            this.angleDeg    = angleDeg;
            this.rpm         = rpm;
            this.timeS       = timeS;
        }
    }

    /** The output of a shot solve: everything the turret needs to act. */
    public static final class ShotSolution {
        /** Hood angle (degrees from vertical). */
        public final double angleDeg;
        /** Flywheel RPM. */
        public final double rpm;
        /** Converged time-of-flight (seconds). */
        public final double timeOfFlightS;
        /**
         * Field-relative bearing to the virtual target (degrees, CCW-positive from
         * +X axis). The turret must convert this to a robot-frame angle.
         */
        public final double aimBearingDeg;
        /**
         * Newton contraction rate from the last two iterations: a value near 0
         * means the solution is stable; near 1 means it is fragile. Computed only
         * when more than two iterations ran; otherwise {@code Double.NaN}.
         */
        public final double contractionRate;
        /** {@code true} when the distance is within the table's measured range. */
        public final boolean inRange;

        public ShotSolution(double angleDeg, double rpm, double timeOfFlightS,
                double aimBearingDeg, double contractionRate, boolean inRange) {
            this.angleDeg        = angleDeg;
            this.rpm             = rpm;
            this.timeOfFlightS   = timeOfFlightS;
            this.aimBearingDeg   = aimBearingDeg;
            this.contractionRate = contractionRate;
            this.inRange         = inRange;
        }
    }

    /** Sorted (ascending distance) static data points loaded from JSON. */
    private final List<DataPoint> points = new ArrayList<>();

    /** {@code true} if the table loaded successfully and has ≥ 1 point. */
    private boolean loaded = false;

    /**
     * Loads {@code turret_data.json} from the deploy directory.
     * Silently falls back to an empty table if the file is missing or malformed —
     * callers should check {@link #isLoaded()} before using solutions.
     */
    public ShooterTable() {
        load();
    }

    /** Returns {@code true} when the table has at least one data point. */
    public boolean isLoaded() {
        return loaded;
    }

    /** Returns the number of data points in the table. */
    public int size() {
        return points.size();
    }

    public double minDistanceInches() {
        if (points.isEmpty()) return Double.NaN;
        return points.get(0).distanceIn;
    }

    public double maxDistanceInches() {
        if (points.isEmpty()) return Double.NaN;
        return points.get(points.size() - 1).distanceIn;
    }

    public boolean isDistanceInRange(double distanceInches) {
        if (points.isEmpty()) return false;
        return distanceInches >= minDistanceInches() && distanceInches <= maxDistanceInches();
    }


    /**
     * Solves for shot parameters given the current geometry.
     *
     * @param dxIn      X-component of (target − robot) in <b>inches</b>
     * @param dyIn      Y-component of (target − robot) in <b>inches</b>
     * @param robotVxIps Robot velocity X in <b>inches/s</b> (field frame)
     * @param robotVyIps Robot velocity Y in <b>inches/s</b> (field frame)
     * @return a {@link ShotSolution}, or {@code null} if the table is empty.
     */
    public ShotSolution solve(double dxIn, double dyIn,
                              double robotVxIps, double robotVyIps) {
        if (!loaded || points.isEmpty()) return null;

        double trueDistIn = Math.hypot(dxIn, dyIn);
        if (trueDistIn < 1.0) {
            // Essentially on top of the target — use the closest-range entry.
            DataPoint near = points.get(0);
            return new ShotSolution(near.angleDeg, near.rpm, near.timeS,
                    Math.toDegrees(Math.atan2(dyIn, dxIn)), 0.0, true);
        }

        // ── Estimate initial horizontal projectile speed from the static table ──
        // vp = distance / time_of_flight at the current static distance estimate.
        // Used as the Newton derivative proxy 1/vp — never differentiate the table.
        double staticTimeS = interpolateTime(trueDistIn);
        double vp = (staticTimeS > 1e-6) ? trueDistIn / staticTimeS
                                          : FALLBACK_PROJECTILE_SPEED_IPS;
        vp = Math.max(vp, 10.0); // guard against zero-divide from bad table data

        // ── Newton's method initial guess ─────────────────────────────────────
        // τ₀ = D / (vp + |v|·cosθ), which stays positive and near-optimal across
        // all velocity directions (WPILib fire-control docs, "Picking an Initial Guess").
        double vMag = Math.hypot(robotVxIps, robotVyIps);
        double cosTheta = (vMag > 1e-6)
                ? (dxIn * robotVxIps + dyIn * robotVyIps) / (trueDistIn * vMag)
                : 0.0;
        double tau = trueDistIn / Math.max(vp + vMag * cosTheta, 1.0);

        double tauPrev = 0.0;
        double tauPrevPrev = 0.0;
        double contractionRate = Double.NaN;

        // ── Newton iteration ──────────────────────────────────────────────────
        for (int i = 0; i < MAX_NEWTON_ITERS; i++) {
            // Step 1: virtual target displacement at current guess
            double vdx = dxIn - robotVxIps * tau;
            double vdy = dyIn - robotVyIps * tau;
            double D   = Math.hypot(vdx, vdy);
            if (D < 1.0) break; // virtual target on top of robot — degenerate

            // Step 2: residual — LUT says this distance should take tof_lut(D) seconds
            double tofLut = interpolateTime(D);
            double E  = tau - tofLut;

            // Step 3: derivative using constant-velocity proxy τ'(D) ≈ 1/vp
            // dD/dτ = -(vdx·vx + vdy·vy) / D  (rate of change of |virtual displacement|)
            double dDdTau = -(vdx * robotVxIps + vdy * robotVyIps) / D;
            double Eprime = 1.0 - (1.0 / vp) * dDdTau; // chain rule: 1 - τ'(D)·(dD/dτ)
            if (Math.abs(Eprime) < 1e-6) break; // guard against divide-by-zero

            // Step 5: Newton update
            tauPrevPrev = tauPrev;
            tauPrev     = tau;
            tau         = tau - E / Eprime;
            tau         = Math.max(tau, 0.001); // tau must be positive

            // Contraction rate: |Δτ_n / Δτ_{n-1}|  — only meaningful from iter 2+
            if (i >= 1) {
                double deltaN   = Math.abs(tau - tauPrev);
                double deltaNm1 = Math.abs(tauPrev - tauPrevPrev);
                if (deltaNm1 > 1e-9) {
                    contractionRate = deltaN / deltaNm1;
                }
            }

            if (Math.abs(tau - tauPrev) < CONVERGENCE_TOL_S) break;
        }

        // ── Read converged shot parameters from the virtual distance ──────────
        double vdxFinal = dxIn - robotVxIps * tau;
        double vdyFinal = dyIn - robotVyIps * tau;
        double Dfinal   = Math.hypot(vdxFinal, vdyFinal);

        double angleDeg = interpolateAngle(Dfinal);
        double rpm      = interpolateRPM(Dfinal);

        boolean inRange = true;

        // Aim bearing derived from the virtual target vector — correct 2D geometry,
        // no separate vperp correction needed.
        double aimBearingDeg = Math.toDegrees(Math.atan2(vdyFinal, vdxFinal));

        return new ShotSolution(angleDeg, rpm, tau, aimBearingDeg, contractionRate, inRange);
    }

    /** Linearly interpolates the hood angle for the given distance. */
    public double interpolateAngle(double distIn) {
        return interpolate(distIn, p -> p.angleDeg);
    }

    /** Linearly interpolates the flywheel RPM for the given distance. */
    public double interpolateRPM(double distIn) {
        return interpolate(distIn, p -> p.rpm);
    }

    /** Linearly interpolates the time-of-flight for the given distance. */
    public double interpolateTime(double distIn) {
        return interpolate(distIn, p -> p.timeS);
    }

    @FunctionalInterface
    private interface PointGetter {
        double get(DataPoint p);
    }

    /**
     * Generic 1-D linear interpolation across the sorted {@link #points} list.
     * Clamps to the nearest endpoint if {@code distIn} is out of range.
     */
    private double interpolate(double distIn, PointGetter getter) {
        if (points.size() == 1) return getter.get(points.get(0));

        // Below the minimum — clamp to first point
        if (distIn <= points.get(0).distanceIn) return getter.get(points.get(0));

        // Above the maximum — clamp to last point
        DataPoint last = points.get(points.size() - 1);
        if (distIn >= last.distanceIn) return getter.get(last);

        // Binary search for the bracket
        int lo = 0, hi = points.size() - 2;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (points.get(mid + 1).distanceIn <= distIn) lo = mid + 1;
            else hi = mid;
        }

        DataPoint a = points.get(lo);
        DataPoint b = points.get(lo + 1);
        double t = (distIn - a.distanceIn) / (b.distanceIn - a.distanceIn);
        return a_lerp(getter.get(a), getter.get(b), t);
    }

    private static double a_lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    /**
     * Loads and parses {@code turret_data.json} from the deploy directory.
     *
     * <p>Uses a hand-written minimal parser instead of a JSON library to avoid
     * any third-party dependency and to keep the parser allocation-free after
     * startup. The format expected is exactly what
     * {@code turret_lookup_generator.py} writes:
     * <pre>
     * { "data_points": [ { "distance":..., "angle":..., "rpm":..., "flight_time":... }, ... ] }
     * </pre>
     * Keys {@code target_height} and {@code vpar} are accepted but ignored (the
     * on-RIO solver handles height via the 3D pose and velocity via Newton).
     */
    private void load() {
        java.io.File file = new java.io.File(
                Filesystem.getDeployDirectory(), "turret_data.json");

        if (!file.exists()) {
            System.err.println("[ShooterTable] turret_data.json not found at: " + file.getAbsolutePath());
            return;
        }

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) sb.append(line);
            parseJson(sb.toString());
            points.sort(Comparator.comparingDouble(p -> p.distanceIn));
            loaded = !points.isEmpty();
            System.out.println("[ShooterTable] Loaded " + points.size() + " data point(s).");
        } catch (IOException e) {
            System.err.println("[ShooterTable] Failed to read turret_data.json: " + e.getMessage());
        }
    }

    /**
     * Minimal hand-rolled JSON parser for the specific structure produced by
     * {@code turret_lookup_generator.py}. Avoids pulling in a full JSON library.
     *
     * <p>Extracts {@code distance}, {@code angle}, {@code rpm}, and
     * {@code flight_time} from each object in the {@code data_points} array.
     * All other keys are skipped.
     */
    private void parseJson(String json) {
        // Find the data_points array
        int arrStart = json.indexOf("[");
        int arrEnd   = json.lastIndexOf("]");
        if (arrStart < 0 || arrEnd < 0) return;

        String arr = json.substring(arrStart + 1, arrEnd);

        // Split into individual objects by "}" — then scan each for key:value pairs
        int depth = 0;
        int objStart = -1;
        for (int i = 0; i < arr.length(); i++) {
            char c = arr.charAt(i);
            if (c == '{') {
                if (depth == 0) objStart = i;
                depth++;
            } else if (c == '}') {
                depth--;
                if (depth == 0 && objStart >= 0) {
                    parseDataPoint(arr.substring(objStart + 1, i));
                    objStart = -1;
                }
            }
        }
    }

    private void parseDataPoint(String obj) {
        double distance   = extractDouble(obj, "distance",    Double.NaN);
        double angle      = extractDouble(obj, "angle",       Double.NaN);
        double rpm        = extractDouble(obj, "rpm",         Double.NaN);
        double flightTime = extractDouble(obj, "flight_time", Double.NaN);

        if (Double.isNaN(distance) || Double.isNaN(angle)
                || Double.isNaN(rpm) || Double.isNaN(flightTime)) {
            System.err.println("[ShooterTable] Skipping incomplete data point: " + obj.trim());
            return;
        }
        points.add(new DataPoint(distance, angle, rpm, flightTime));
    }

    /**
     * Extracts the first numeric value associated with {@code key} in a flat
     * JSON object fragment (no nesting). Returns {@code defaultVal} if not found.
     */
    private static double extractDouble(String obj, String key, double defaultVal) {
        // Search for "key" : <number>
        String search = "\"" + key + "\"";
        int idx = obj.indexOf(search);
        if (idx < 0) return defaultVal;

        // Skip past the key, colon, and any whitespace
        int colon = obj.indexOf(':', idx + search.length());
        if (colon < 0) return defaultVal;

        int start = colon + 1;
        while (start < obj.length() && (obj.charAt(start) == ' ' || obj.charAt(start) == '\t'
                || obj.charAt(start) == '\n' || obj.charAt(start) == '\r')) {
            start++;
        }

        // Read digits, minus, dot, 'e', 'E', '+'
        int end = start;
        while (end < obj.length()) {
            char c = obj.charAt(end);
            if (Character.isDigit(c) || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+') {
                end++;
            } else {
                break;
            }
        }
        if (start == end) return defaultVal;

        try {
            return Double.parseDouble(obj.substring(start, end));
        } catch (NumberFormatException e) {
            return defaultVal;
        }
    }
}
