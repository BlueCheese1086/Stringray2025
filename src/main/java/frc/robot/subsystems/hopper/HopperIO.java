package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        double trackPercent = 0;
        Voltage trackVoltage = Volts.zero();
        Current trackCurrent = Amps.zero();
        Temperature trackTemperature = Celsius.zero();
    }

    public void updateInputs(HopperIOInputs inputs);

    public void setPercent(double percent);

    public void setVoltage(Voltage voltage);
}