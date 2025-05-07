package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public Current getCurrent() {
        return Amps.zero();
    }

    public double getPercent() {
        return 0;
    }

    public Temperature getTemperature() {
        return Celsius.zero();
    }

    public Voltage getVoltage() {
        return Volts.zero();
    }

    public Distance getLaserReading() {
        return Inches.zero();
    }

    public int getLaserStatus() {
        return LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    public void setPercent(double percent) {}

    public void setVoltage(Voltage voltage) {}
}