package frc.robot.utils.targeting;

/**
 * Per-turret-angle shot corrections. The kicker and the turret put different spin on the ball
 * depending on which way the turret points relative to the robot, so the same solved shot lands
 * short or to one side at some turret angles. This table holds a range trim and an aim trim at
 * {@link #POINTS} turret angles spaced {@link #SPACING_DEG} apart, starting at -180 (the front of
 * the robot), and interpolates linearly between them, wrapping at +/-180.
 *
 * <p>Turret angles use the {@link TurretAngles} convention: clockwise-positive, 0 = back of robot.
 */
public final class TurretAngleTrims {
    /** Number of table points. */
    public static final int POINTS = 8;
    /** Degrees between table points. */
    public static final double SPACING_DEG = 360.0 / POINTS;
    /** Turret angle of the first table point. */
    public static final double FIRST_ANGLE_DEG = -180.0;

    private TurretAngleTrims() {}

    /**
     * @param distanceIn inches added to the solved distance (positive = shoot farther)
     * @param yawDeg     degrees added to the turret angle (positive = aim clockwise, to the
     *                   shooter's right)
     */
    public record Trim(double distanceIn, double yawDeg) {
        public static final Trim NONE = new Trim(0.0, 0.0);
    }

    /** Turret angle of table point {@code index}. */
    public static double angleOfPoint(int index) {
        return FIRST_ANGLE_DEG + index * SPACING_DEG;
    }

    /**
     * Interpolates the trims at a turret angle.
     *
     * @param turretAngleDeg turret angle (any range; wrapped into [-180, 180))
     * @param distanceTrimIn {@link #POINTS} range trims, inches, at {@link #angleOfPoint}
     * @param yawTrimDeg     {@link #POINTS} aim trims, degrees, at {@link #angleOfPoint}
     * @return the interpolated trim, or {@link Trim#NONE} for a non-finite angle
     */
    public static Trim at(double turretAngleDeg, double[] distanceTrimIn, double[] yawTrimDeg) {
        if (distanceTrimIn.length != POINTS || yawTrimDeg.length != POINTS) {
            throw new IllegalArgumentException("Turret angle trim tables need " + POINTS + " points");
        }
        if (!Double.isFinite(turretAngleDeg)) return Trim.NONE;

        double sinceFirst = TurretAngles.normalize180(turretAngleDeg) - FIRST_ANGLE_DEG;
        int lower = Math.min((int) Math.floor(sinceFirst / SPACING_DEG), POINTS - 1);
        int upper = (lower + 1) % POINTS;
        double t = (sinceFirst - lower * SPACING_DEG) / SPACING_DEG;
        return new Trim(
            lerp(distanceTrimIn[lower], distanceTrimIn[upper], t),
            lerp(yawTrimDeg[lower], yawTrimDeg[upper], t));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
