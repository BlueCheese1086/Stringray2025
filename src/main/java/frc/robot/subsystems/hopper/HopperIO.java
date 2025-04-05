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
        double percent = 0;
        Voltage voltage = Volts.zero();
        Current current = Amps.zero();
        Temperature temperature = Celsius.zero();

        Distance laserReading = Meters.zero();
        int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Updates a set of IOInputs. */
    public void updateInputs(HopperIOInputs inputs);

    /** Sets the percent output of the motor. */
    public void setPercent(double percent);

    /** Sets the voltage output of the motor. */
    public void setVoltage(Voltage voltage);
}