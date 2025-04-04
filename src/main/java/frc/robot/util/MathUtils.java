package frc.robot.util;

public class MathUtils {
    /**
     * Applies a deadband and offsets the output by the deadband.
     * This allows you to still reach that lower range you would normally lose when using a normal deadband.
     * 
     * @param value The number to apply a deadband to.
     * @param deadband The deadband.
     */
    public static double applyDeadbandWithOffsets(double value, double deadband) {
        if (Math.abs(value) < Math.abs(deadband)) return 0;

        return value - Math.copySign(deadband, value);
    }

    /**
     * Applies a deadband and offsets the output by the deadband.
     * This allows you to still reach that lower range you would normally lose when using a normal deadband.
     * However, it also allows you to run at the higher range output that would be lost by the offset.
     * Once the value exceeds vanillaVal, the offset is no longer applied to it.
     * 
     * @param value The number to apply a deadband to.
     * @param deadband The deadband.
     * @param vanillaVal The value where the offset is no longer applied.
     */
    public static double applyDeadbandWithOffsets(double value, double deadband, double vanillaVal) {
        if (Math.abs(value) < Math.abs(deadband)) return 0;

        if (Math.abs(value) >= Math.abs(vanillaVal)) return value;

        return value - Math.copySign(deadband, value);
    }

    /**
     * Checks if a number is within a deadband.
     * 
     * @param value The number to check.
     * @param deadband The deadband.
     */
    public static boolean withinDeadband(double value, double deadband) {
        return Math.abs(value) < Math.abs(deadband);
    }
}