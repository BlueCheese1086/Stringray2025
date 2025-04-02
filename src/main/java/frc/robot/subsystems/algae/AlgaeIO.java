package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public class AlgaeIOInputs {
        public double percent = 0;
        public Voltage voltage = Volts.zero();
        public Current current = Amps.zero();
        public Temperature temperature = Celsius.zero();

        public Distance laserReading = Meters.zero();
        public int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    }

    /** Updates a set of {@link AlgaeIOInputs} with new values. */
    public void updateInputs(AlgaeIOInputs inputs);

    /** Sets the percent output of the motor. */
    public void setPercent(double percent);

    /** Sets the voltage output of the motor. */
    public void setVoltage(Voltage voltage);
}