package frc.robot.subsystems.hopper;

import au.grapplerobotics.LaserCan;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        double current = 0;
        double percent = 0;
        double temperature = 0;
        double voltage = 0;

        double laserReading = 0;
        int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Updates a set of {@link HopperIOInputs}. */
    public void updateInputs(HopperIOInputs inputs);

    /** Sets the percent output of the IO. */
    public void setPercent(double percent);

    /** Sets the voltage output of the IO. */
    public void setVoltage(double voltage);
}