package frc.robot.utils;

public class MathUtils {
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (((value - inMin) / (inMax - inMin)) * (outMax - outMin)) + outMin;
      }

    public static boolean withinRange(double value, double minimum, double maximum) {
      return minimum <= value && value <= maximum;
    }
}