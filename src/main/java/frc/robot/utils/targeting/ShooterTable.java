package frc.robot.utils.targeting;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The empirical lookup table from {@code deploy/turret_data.json}: hood angle, flywheel RPM and
 * flight time measured for static shots at the hub, linearly interpolated by distance.
 *
 * <p>Outside the measured distances the values clamp to the nearest end point. A solution counts as
 * in range within {@code rangeMarginIn} of either end. The table ignores target height: every point
 * was recorded against the hub.
 *
 * <p>Distances in the file are inches; hood angles are true degrees from vertical.
 */
public final class ShooterTable implements ShotSolver {

    /** One static shot from {@code turret_data.json}. */
    public record DataPoint(double distanceIn, double angleDeg, double rpm, double timeS) {}

    private final List<DataPoint> points;
    private final double rangeMarginIn;

    /** Loads {@code turret_data.json} from the deploy directory. An unreadable file gives an empty table. */
    public ShooterTable(double rangeMarginIn) {
        this(loadDeployFile(), rangeMarginIn);
    }

    /** Builds a table from points in any order. Invalid and duplicate-distance points are dropped. */
    public ShooterTable(List<DataPoint> rawPoints, double rangeMarginIn) {
        this.points = Collections.unmodifiableList(validate(rawPoints));
        this.rangeMarginIn = rangeMarginIn;
    }

    /** True when the table has at least one point. */
    public boolean isLoaded() {
        return !points.isEmpty();
    }

    public int size() {
        return points.size();
    }

    public double minDistanceInches() {
        return points.isEmpty() ? Double.NaN : points.get(0).distanceIn();
    }

    public double maxDistanceInches() {
        return points.isEmpty() ? Double.NaN : points.get(points.size() - 1).distanceIn();
    }

    /** True when the distance is within the measured range, widened by the range margin on each side. */
    public boolean isDistanceInRange(double distanceIn) {
        if (points.isEmpty()) return false;
        return distanceIn >= minDistanceInches() - rangeMarginIn
            && distanceIn <= maxDistanceInches() + rangeMarginIn;
    }

    @Override
    public ShotSolution solve(double distanceM, double targetHeightM) {
        if (points.isEmpty()) return null;
        double distanceIn = Units.metersToInches(distanceM);
        return new ShotSolution(
            interpolateAngle(distanceIn),
            interpolateRPM(distanceIn),
            interpolateTime(distanceIn),
            isDistanceInRange(distanceIn),
            ShotSolution.Source.LUT);
    }

    /** Hood angle (degrees from vertical) at the distance, clamped at the table ends. */
    public double interpolateAngle(double distanceIn) {
        return interpolate(distanceIn, DataPoint::angleDeg);
    }

    /** Flywheel RPM at the distance, clamped at the table ends. */
    public double interpolateRPM(double distanceIn) {
        return interpolate(distanceIn, DataPoint::rpm);
    }

    /** Flight time (seconds) at the distance, clamped at the table ends. */
    public double interpolateTime(double distanceIn) {
        return interpolate(distanceIn, DataPoint::timeS);
    }

    @FunctionalInterface
    private interface PointGetter {
        double get(DataPoint p);
    }

    private double interpolate(double distanceIn, PointGetter getter) {
        if (points.isEmpty()) return Double.NaN;
        DataPoint first = points.get(0);
        DataPoint last = points.get(points.size() - 1);
        if (distanceIn <= first.distanceIn()) return getter.get(first);
        if (distanceIn >= last.distanceIn()) return getter.get(last);

        // Binary search for the bracketing pair.
        int lo = 0, hi = points.size() - 2;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (points.get(mid + 1).distanceIn() <= distanceIn) lo = mid + 1;
            else hi = mid;
        }
        DataPoint a = points.get(lo);
        DataPoint b = points.get(lo + 1);
        double t = (distanceIn - a.distanceIn()) / (b.distanceIn() - a.distanceIn());
        return getter.get(a) + t * (getter.get(b) - getter.get(a));
    }

    /** Sorts by distance and drops points with non-finite or non-positive values, or a repeated distance. */
    private static List<DataPoint> validate(List<DataPoint> rawPoints) {
        List<DataPoint> sorted = new ArrayList<>();
        for (DataPoint p : rawPoints) {
            boolean valid = Double.isFinite(p.distanceIn()) && p.distanceIn() > 0
                && Double.isFinite(p.angleDeg())
                && Double.isFinite(p.rpm()) && p.rpm() > 0
                && Double.isFinite(p.timeS()) && p.timeS() > 0;
            if (valid) {
                sorted.add(p);
            } else {
                System.err.println("[ShooterTable] Skipping invalid data point: " + p);
            }
        }
        sorted.sort(Comparator.comparingDouble(DataPoint::distanceIn));

        List<DataPoint> result = new ArrayList<>();
        for (DataPoint p : sorted) {
            if (!result.isEmpty() && result.get(result.size() - 1).distanceIn() == p.distanceIn()) {
                System.err.println("[ShooterTable] Skipping duplicate distance: " + p);
                continue;
            }
            result.add(p);
        }
        return result;
    }

    private static List<DataPoint> loadDeployFile() {
        File file = new File(Filesystem.getDeployDirectory(), "turret_data.json");
        if (!file.exists()) {
            System.err.println("[ShooterTable] turret_data.json not found at: " + file.getAbsolutePath());
            return List.of();
        }
        try {
            List<DataPoint> points = parse(Files.readString(file.toPath()));
            System.out.println("[ShooterTable] Loaded " + points.size() + " data point(s).");
            return points;
        } catch (IOException e) {
            System.err.println("[ShooterTable] Failed to read turret_data.json: " + e.getMessage());
            return List.of();
        }
    }

    /**
     * Minimal parser for the structure {@code turret_lookup_generator.py} writes:
     * {@code { "data_points": [ { "distance":..., "angle":..., "rpm":..., "flight_time":... }, ... ] }}.
     * Other keys ({@code target_height}, {@code vpar}) are ignored. Hand-rolled because initializing
     * Jackson takes seconds on a roboRIO.
     */
    static List<DataPoint> parse(String json) {
        List<DataPoint> points = new ArrayList<>();
        int arrStart = json.indexOf('[');
        int arrEnd = json.lastIndexOf(']');
        if (arrStart < 0 || arrEnd < arrStart) return points;

        String arr = json.substring(arrStart + 1, arrEnd);
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
                    DataPoint p = parseDataPoint(arr.substring(objStart + 1, i));
                    if (p != null) points.add(p);
                    objStart = -1;
                }
            }
        }
        return points;
    }

    private static DataPoint parseDataPoint(String obj) {
        double distance = extractDouble(obj, "distance");
        double angle = extractDouble(obj, "angle");
        double rpm = extractDouble(obj, "rpm");
        double flightTime = extractDouble(obj, "flight_time");
        if (Double.isNaN(distance) || Double.isNaN(angle) || Double.isNaN(rpm) || Double.isNaN(flightTime)) {
            System.err.println("[ShooterTable] Skipping incomplete data point: " + obj.trim());
            return null;
        }
        return new DataPoint(distance, angle, rpm, flightTime);
    }

    /** The number after {@code "key":} in a flat JSON object fragment, or NaN if absent. */
    private static double extractDouble(String obj, String key) {
        String search = "\"" + key + "\"";
        int idx = obj.indexOf(search);
        if (idx < 0) return Double.NaN;
        int colon = obj.indexOf(':', idx + search.length());
        if (colon < 0) return Double.NaN;

        int start = colon + 1;
        while (start < obj.length() && Character.isWhitespace(obj.charAt(start))) start++;
        int end = start;
        while (end < obj.length()) {
            char c = obj.charAt(end);
            if (Character.isDigit(c) || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+') end++;
            else break;
        }
        if (start == end) return Double.NaN;
        try {
            return Double.parseDouble(obj.substring(start, end));
        } catch (NumberFormatException e) {
            return Double.NaN;
        }
    }
}
