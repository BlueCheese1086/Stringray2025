package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Algae extends SubsystemBase {
    private AlgaeIO io;
    private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

    public Algae(AlgaeIO io) {
        this.io = io;
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
    }

    public double getPercent() {
        return inputs.percent;
    }

    public Voltage getVoltage() {
        return inputs.voltage;
    }

    public Current getCurrent() {
        return inputs.current;
    }

    public Temperature getTemperature() {
        return inputs.temperature;
    }

    public Distance getLaserReading() {
        return inputs.laserReading;
    }

    public int getLaserStatus() {
        return inputs.laserStatus;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Algae", inputs);
    }
}