package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Measures where the Quest sits on the robot (the translation of {@code ROBOT_TO_QUEST}) from Quest
 * poses recorded while the robot spins in place. The Quest traces a circle around the robot's center;
 * a least-squares circle fit finds that center, and each sample's Quest position relative to it,
 * rotated into the robot frame, gives the offset. Only relative motion is used, so a wrong offset in
 * the current pose reset doesn't bias the result.
 */
public final class QuestOffsetCalibrator {
    private static final int MIN_SAMPLES = 50;
    /** At least half a turn, or the circle fit is poorly constrained. */
    private static final double MIN_ROTATION_RAD = Math.PI;

    private final double mountYawRad;
    private final List<Pose2d> samples = new ArrayList<>();
    private double unwrappedHeading, minHeading, maxHeading;

    /** @param mountYawRad yaw of the Quest relative to the robot (the rotation in {@code ROBOT_TO_QUEST}) */
    public QuestOffsetCalibrator(double mountYawRad) {
        this.mountYawRad = mountYawRad;
    }

    public void reset() {
        samples.clear();
    }

    /** @param questFieldPose the Quest's own pose on the field (not the robot pose derived from it) */
    public void addSample(Pose2d questFieldPose) {
        double heading = questFieldPose.getRotation().getRadians();
        if (samples.isEmpty()) {
            unwrappedHeading = minHeading = maxHeading = heading;
        } else {
            double previous = samples.get(samples.size() - 1).getRotation().getRadians();
            unwrappedHeading += MathUtil.angleModulus(heading - previous);
            minHeading = Math.min(minHeading, unwrappedHeading);
            maxHeading = Math.max(maxHeading, unwrappedHeading);
        }
        samples.add(questFieldPose);
    }

    /** @return the Quest's position in the robot frame (meters), or empty if there isn't enough data yet */
    public Optional<Translation2d> solve() {
        if (samples.size() < MIN_SAMPLES || maxHeading - minHeading < MIN_ROTATION_RAD) return Optional.empty();

        // Kasa circle fit on mean-centered points: x^2 + y^2 + D x + E y + F = 0, least squares in D, E, F.
        double meanX = 0, meanY = 0;
        for (Pose2d p : samples) {
            meanX += p.getX();
            meanY += p.getY();
        }
        meanX /= samples.size();
        meanY /= samples.size();
        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
        for (Pose2d p : samples) {
            double x = p.getX() - meanX, y = p.getY() - meanY, z = x * x + y * y;
            sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
            sxz += x * z; syz += y * z; sz += z;
        }
        double n = samples.size();
        double[] def = solve3(
            new double[][] {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, n}},
            new double[] {-sxz, -syz, -sz});
        if (def == null) return Optional.empty();
        Translation2d center = new Translation2d(meanX - def[0] / 2, meanY - def[1] / 2);

        // Quest position relative to the center, rotated into the robot frame, averaged over samples.
        double offsetX = 0, offsetY = 0;
        for (Pose2d p : samples) {
            double robotHeading = p.getRotation().getRadians() - mountYawRad;
            Translation2d inRobot = p.getTranslation().minus(center).rotateBy(new Rotation2d(-robotHeading));
            offsetX += inRobot.getX();
            offsetY += inRobot.getY();
        }
        return Optional.of(new Translation2d(offsetX / n, offsetY / n));
    }

    /** A one-line result for the console. */
    public String describe() {
        double turnedDeg = samples.isEmpty() ? 0.0 : Math.toDegrees(maxHeading - minHeading);
        return solve()
            .map(t -> String.format("[QuestCal] ROBOT_TO_QUEST x = %.2f in, y = %.2f in (%d samples, %.0f deg turned)",
                Units.metersToInches(t.getX()), Units.metersToInches(t.getY()), samples.size(), turnedDeg))
            .orElse(String.format("[QuestCal] not enough data: %d samples, %.0f deg turned (need %d and 180)",
                samples.size(), turnedDeg, MIN_SAMPLES));
    }

    /** Solves a 3x3 linear system by Cramer's rule; null if singular. */
    private static double[] solve3(double[][] a, double[] b) {
        double det = det3(a);
        if (Math.abs(det) < 1e-12) return null;
        double[] result = new double[3];
        for (int col = 0; col < 3; col++) {
            double[][] m = {a[0].clone(), a[1].clone(), a[2].clone()};
            for (int row = 0; row < 3; row++) m[row][col] = b[row];
            result[col] = det3(m) / det;
        }
        return result;
    }

    private static double det3(double[][] m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
}
