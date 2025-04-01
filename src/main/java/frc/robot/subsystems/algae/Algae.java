package frc.robot.subsystems.algae;

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }
}