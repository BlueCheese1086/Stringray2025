package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        Current current = Amps.zero();
        double percent = 0;
        Temperature temperature = Celsius.zero();
        Voltage voltage = Volts.zero();

        Distance laserReading = Meters.zero();
        int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Updates a set of {@link HopperIOInputs}. */
    public void updateInputs(HopperIOInputs inputs);

    /** Sets the percent output of the IO. */
    public void setPercent(double percent);

    /** Sets the voltage output of the IO. */
    public void setVoltage(Voltage voltage);
}