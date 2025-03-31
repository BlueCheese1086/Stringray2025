package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralIOSim implements CoralIO {
    private DCMotorSim algaeMotorSim;
    private DCMotorSim coralMotorSim;

    /** Creates a simulated version of the carriage. */
    public CoralIOSim() {
        algaeMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
        coralMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        algaeMotorSim.update(0.02);
        coralMotorSim.update(0.02);

        inputs.percent = coralMotorSim.getInputVoltage() / RobotController.getInputVoltage();
        inputs.voltage = Volts.of(coralMotorSim.getInputVoltage());
        inputs.current = Amps.of(coralMotorSim.getCurrentDrawAmps());
    }


    @Override
    public void setPercent(double percent) {
        coralMotorSim.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        coralMotorSim.setInputVoltage(voltage.in(Volts));
    }
}