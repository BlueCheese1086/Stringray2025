package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    protected Coral() {}

    /** Sets the percent output of the coral roller. */
    public void setPercent(double percent) {}

    /** Sets the voltage of the coral roller. */
    public void setVoltage(Voltage voltage) {}

    /** Gets the proximity read by the CANandColor. */
    public double getSensorProximity() {
        return -1;
    }

    /** Gets the color seen by the CANandColor. */
    public String getSensorColor() {
        return "";
    }

    /** Gets the distance seen by the LaserCan. */
    public Distance getLaserReading() {
        return Meters.of(Double.MAX_VALUE);
    }

    /** Gets the latest status of the LaserCan. */
    public int getLaserStatus() {
        return LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Gets the percent output of the coral motor. */
    public double getPercent() {
        return 0;
    }

    /** Gets the voltage output of the coral motor. */
    public Voltage getVoltage() {
        return Volts.zero();
    }

    /** Gets the current applied to the coral motor. */
    public Current getCurrent() {
        return Amps.zero();
    }

    /** Gets the internal temperature of the coral motor. */
    public Temperature getTemperature() {
        return Celsius.zero();
    }
}