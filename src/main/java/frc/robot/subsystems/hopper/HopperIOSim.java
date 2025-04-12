package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
    private DCMotorSim track;

    public HopperIOSim() {
        track = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        track.update(0.02);

        inputs.current = track.getCurrentDrawAmps();
        inputs.percent = track.getInputVoltage() / RobotController.getInputVoltage();
        inputs.voltage = track.getInputVoltage();
    }

    @Override
    public void setPercent(double percent) {
        track.setInput(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(double voltage) {
        track.setInput(voltage);
    }
}