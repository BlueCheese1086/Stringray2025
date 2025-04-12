package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    public HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Hopper", inputs);
    }

    public void setPercent(double percent) {
        Logger.recordOutput("/Hopper/PercentSetpoint", percent);
        io.setPercent(percent);
    }

    public void setVoltage(Voltage voltage) {
        Logger.recordOutput("/Hopper/VoltageSetpoint", voltage);
        io.setVoltage(voltage);
    }
}