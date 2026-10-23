package frc.robot.utils.targeting;

/**
 * Turret angle conventions in one place. The turret angle is clockwise-positive, measured from the
 * back of the robot. Field bearings and robot headings are WPILib's counter-clockwise-positive.
 */
public final class TurretAngles {
    private TurretAngles() {}

    /** Wraps an angle in degrees into [-180, 180). */
    public static double normalize180(double deg) {
        return deg - 360.0 * Math.floor((deg + 180.0) / 360.0);
    }

    /**
     * Turret angle that points along a field bearing.
     *
     * @param robotHeadingRad robot heading, CCW-positive
     * @param fieldBearingRad direction to aim, CCW-positive from the field +X axis
     * @return turret angle in degrees, CW-positive from the back of the robot, in [-180, 180)
     */
    public static double turretAngleForBearingDeg(double robotHeadingRad, double fieldBearingRad) {
        // Robot-relative bearing (CCW) is bearing - heading; flip to CW and move zero to the back.
        return normalize180(Math.toDegrees(robotHeadingRad) - Math.toDegrees(fieldBearingRad) + 180.0);
    }

    /**
     * Returns an angle equivalent to {@code deg} (mod 360) that lies within the turret's travel. An
     * angle already within travel is returned unchanged. If the travel is under 360 degrees and no
     * equivalent fits, returns the nearer limit.
     */
    public static double wrapIntoTravel(double deg, double minDeg, double maxDeg) {
        if (deg >= minDeg && deg <= maxDeg) return deg;
        double offset = deg - minDeg;
        double wrapped = minDeg + (offset - 360.0 * Math.floor(offset / 360.0));
        if (wrapped <= maxDeg) return wrapped;
        return (wrapped - maxDeg <= minDeg + 360.0 - wrapped) ? maxDeg : minDeg;
    }
}
