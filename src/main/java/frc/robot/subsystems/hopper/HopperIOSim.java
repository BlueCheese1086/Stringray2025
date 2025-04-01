package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
    private DCMotorSim hopper;

    public HopperIOSim() {
        this.hopper = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        hopper.update(0.02);
        inputs.trackPercent = hopper.getInputVoltage() / RobotController.getInputVoltage();
        inputs.trackVoltage = Volts.of(hopper.getInputVoltage());
        inputs.trackCurrent = Amps.of(hopper.getCurrentDrawAmps());
    }

    @Override
    public void setPercent(double percent) {
        hopper.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        hopper.setInputVoltage(voltage.in(Volts));
    }
}