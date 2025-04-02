package frc.robot.util;

public class MathUtils {
    public static double applyDeadbandWithOffsets(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;

        return value - Math.copySign(deadband, value);
    }

    public static boolean withinDeadband(double value, double deadband) {
        return Math.abs(value) < Math.abs(deadband);
    }
}