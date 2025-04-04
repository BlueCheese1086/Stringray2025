package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
    private AlgaeIO io;
    private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

    /**
     * Creates a new {@link Algae} subsystem.
     * 
     * @param io The {@link AlgaeIO} to control.
     */
    public Algae(AlgaeIO io) {
        this.io = io;
    }

    /** Sets the percent output of the algae motor. */
    public void setPercent(double percent) {
        Logger.recordOutput("/Algae/GoalPercent", percent);
        io.setPercent(percent);
    }

    /** Sets the voltage output of the algae motor. */
    public void setVoltage(Voltage voltage) {
        Logger.recordOutput("/Algae/GoalVoltage", voltage);
        io.setVoltage(voltage);
    }

    /** Gets the percent output of the algae motor. */
    public double getPercent() {
        return inputs.percent;
    }

    /** Gets the voltage output of the algae motor. */
    public Voltage getVoltage() {
        return inputs.voltage;
    }

    /** Gets the applied current of the algae motor. */
    public Current getCurrent() {
        return inputs.current;
    }

    /** Gets the internal temperature of the algae motor. */
    public Temperature getTemperature() {
        return inputs.temperature;
    }

    /** Gets the distance read by the laser. */
    public Distance getLaserReading() {
        return inputs.laserReading;
    }

    /** Gets the latest status of the laser.  */
    public int getLaserStatus() {
        return inputs.laserStatus;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Algae", inputs);
    }
}