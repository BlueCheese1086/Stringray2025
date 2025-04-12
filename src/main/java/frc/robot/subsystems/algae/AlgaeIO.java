package frc.robot.subsystems.algae;

import au.grapplerobotics.LaserCan;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public class AlgaeIOInputs {
        double current = 0; // Amps
        double percent = 0;
        double temperature = 0; // Celsius
        double voltage = 0; // Volts

        double laserReading = 0;
        int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Updates a set of {@link AlgaeIOInputs} with new values. */
    public void updateInputs(AlgaeIOInputs inputs);

    /** Sets the percent output of the motor. */
    public void setPercent(double percent);

    /** Sets the voltage output of the motor. */
    public void setVoltage(double voltage);
}