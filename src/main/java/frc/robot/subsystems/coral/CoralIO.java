package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public class CoralIOInputs {
        public double percent = 0;
        public Voltage voltage = Volts.zero();
        public Current current = Amps.zero();
        public Temperature temperature = Celsius.zero();

        public double sensorProximity = 0;
        public String sensorColor = "";

        public Distance laserReading = Millimeters.zero();
        public int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /**
     * Updates the inputs parameter with current values.
     * 
     * @param inputs The inputs to update.
     */
    public void updateInputs(CoralIOInputs inputs);

    /** Sets the percent output of the coral roller. */
    public void setPercent(double percent);

    /** Sets the voltage of the coral roller. */
    public void setVoltage(Voltage voltage);
}