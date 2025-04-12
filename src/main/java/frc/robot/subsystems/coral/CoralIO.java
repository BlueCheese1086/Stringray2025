package frc.robot.subsystems.coral;

import au.grapplerobotics.LaserCan;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public class CoralIOInputs {
        public double percent = 0;
        public double voltage = 0; // Volts
        public double current = 0; // Amps
        public double temperature = 0; // Celsius

        public double sensorProximity = 0; // Unknown
        public String sensorColor = "";

        public double laserReading = 0; // Meters
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
    public void setVoltage(double voltage);
}