package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
    private DCMotorSim algae;

    public AlgaeIOSim() {
        this.algae = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.02, 1),
                DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        algae.update(0.02);

        inputs.percent = algae.getInputVoltage() / RobotController.getInputVoltage();
        inputs.voltage = Volts.of(algae.getInputVoltage());
        inputs.current = Amps.of(algae.getCurrentDrawAmps());
    }

    @Override
    public void setPercent(double percent) {
        algae.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        algae.setInputVoltage(voltage.in(Volts));
    }
}