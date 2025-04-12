package frc.robot.subsystems.coral;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralIOSim implements CoralIO {
    private DCMotorSim motorSim;

    /** Creates a simulated version of the coral subsystem. */
    public CoralIOSim() {
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        motorSim.update(0.02);

        inputs.percent = motorSim.getInputVoltage() / RobotController.getInputVoltage();
        inputs.voltage = motorSim.getInputVoltage();
        inputs.current = motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setPercent(double percent) {
        motorSim.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(double voltage) {
        motorSim.setInputVoltage(voltage);
    }
}