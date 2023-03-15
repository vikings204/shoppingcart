package frc.robot.util;

public class Math204 {
    public static PolarCoordinate CartesianToPolar(double x, double y) {
        double r = Math.sqrt(x*x + y*y);
        double rad = Math.atan2(y, x); // returns the angle in radians

        // Convert radians to degrees
        double deg = (Math.toDegrees(rad) + 90)*-1;

        // conv
        /*if (deg > 0) {
            deg = (deg-360);
        }*/

        return new PolarCoordinate(r, deg);
    }

    public int GetQuadrant(int deg, int pDeg) {
        int x = (deg % 360);
        //if ()
        return 69;
    }
}
