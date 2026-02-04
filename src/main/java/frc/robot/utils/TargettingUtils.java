package frc.robot.utils;

public class TargettingUtils {
    public static class ControlTarget {
        public final int rpm;
        public final double hoodAngle;
        public final boolean properlySet;

        public ControlTarget(int rpm, double hoodAngle) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
            this.properlySet = true;
        }

        public ControlTarget() {
            this.rpm = (int) Double.NaN;
            this.hoodAngle = Double.NaN;
            this.properlySet = false;
        }

        public double getRPS() { return rpm/60; }
    }
}
