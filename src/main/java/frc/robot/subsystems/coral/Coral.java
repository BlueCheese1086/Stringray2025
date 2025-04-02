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
     * Creates a new {@link Coral} subsystem.
     * 
     * @param coralIO The {@link CoralIO} to control.
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

    /** Gets the proximity read by the CANandColor. */
    public double getSensorProximity() {
        return inputs.sensorProximity;
    }

    /** Gets the color seen by the CANandColor. */
    public String getSensorColor() {
        return inputs.sensorColor;
    }

    /** Gets the distance seen by the LaserCan. */
    public Distance getLaserReading() {
        return inputs.laserReading;
    }

    /** Gets the latest status of the LaserCan. */
    public int getLaserStatus() {
        return inputs.laserStatus;
    }

    /** Gets the percent output of the coral motor. */
    public double getPercent() {
        return inputs.percent;
    }

    /** Gets the voltage output of the coral motor. */
    public Voltage getVoltage() {
        return inputs.voltage;
    }

    /** Gets the current applied to the coral motor. */
    public Current getCurrent() {
        return inputs.current;
    }

    /** Gets the internal temperature of the coral motor. */
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
        Logger.processInputs("/RealOutputs/Subsystems/Coral", inputs);
    }
}