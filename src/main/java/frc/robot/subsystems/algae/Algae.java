package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    public void setPercent(double percent) {}

    public void setVoltage(Voltage voltage) {}

    public double getPercent() {
        return 0;
    }

    public Voltage getVoltage() {
        return Volts.zero();
    }

    public Current getCurrent() {
        return Amps.zero();
    }

    public Temperature getTemperature() {
        return Celsius.zero();
    }

    public int getLaserStatus() {
        return LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    public Distance getLaserDistance() {
        return Meters.of(Double.MAX_VALUE);
    }
}
