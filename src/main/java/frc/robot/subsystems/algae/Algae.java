package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
    private AlgaeIO io;
    public AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

    /**
     * Creates a new {@link Algae} subsystem.
     * 
     * @param io The {@link AlgaeIO} to control.
     */
    public Algae(AlgaeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Algae", inputs);
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
}