package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
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
        inputs.voltage = Volts.of(motorSim.getInputVoltage());
        inputs.current = Amps.of(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setPercent(double percent) {
        motorSim.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motorSim.setInputVoltage(voltage.in(Volts));
    }
}