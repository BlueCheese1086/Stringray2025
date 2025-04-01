package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    public HopperIO io;
    public HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO io){
        this.io = io;
    }

    public void setPercent(double percent){
        io.setPercent(percent);
    }

    public void setVoltage(Voltage voltage){
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Subsystems/Hopper", inputs);
    }
}