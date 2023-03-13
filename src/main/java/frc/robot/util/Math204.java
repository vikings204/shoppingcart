package frc.robot.util;

public class Math204 {
    public static PolarCoordinate CartesianToPolar(double x, double y) {
        double r = Math.sqrt(x*x + y*y);
        double rad = Math.atan2(y, x); // returns the angle in radians

        // Convert radians to degrees
        double deg = Math.toDegrees(rad);

        // conv
        if (deg > 180) {
            deg = -(deg-180);
        }

        return new PolarCoordinate(r, deg);
    }
}
