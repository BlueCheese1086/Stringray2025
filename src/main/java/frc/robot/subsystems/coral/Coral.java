package frc.robot.subsystems.coral;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
    private CoralIO coralIO;
    private CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    /**
     * Creates a new Coral subsystem.
     * 
     * @param coralIO The coralIO to control.
     */
    public Coral(CoralIO coralIO) {
        this.coralIO = coralIO;
    }

    /** Sets the percent output of the coral roller. */
    public void setPercent(double percent) {
        coralIO.setPercent(percent);
    }

    /** Sets the voltage of the coral roller. */
    public void setVoltage(Voltage voltage) {
        coralIO.setVoltage(voltage);
    }

    public double getSensorProximity() {
        return inputs.sensorProximity;
    }

    public String getSensorColor() {
        return inputs.sensorColor;
    }

    public Distance getLaserReading() {
        return inputs.laserReading;
    }

    public int getLaserStatus() {
        return inputs.laserStatus;
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

    /**
     * Runs once every tick the subsystem is active.
     * 
     * It updates the inputs variable and logs it through AdvantageKit.
     */
    @Override
    public void periodic() {
        coralIO.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Subsystems/Carriage", inputs);
    }
}