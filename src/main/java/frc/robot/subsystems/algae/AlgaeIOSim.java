package frc.robot.subsystems.algae;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
    private DCMotorSim algae;

    public AlgaeIOSim() {
        this.algae = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.02, 1), DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        algae.update(0.02);

        inputs.current = algae.getCurrentDrawAmps();
        inputs.percent = algae.getInputVoltage() / RobotController.getInputVoltage();
        inputs.voltage = algae.getInputVoltage();
    }

    @Override
    public void setPercent(double percent) {
        algae.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(double voltage) {
        algae.setInputVoltage(voltage);
    }
}