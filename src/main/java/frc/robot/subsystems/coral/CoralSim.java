
package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.TurboLogger;

public class CoralSim extends Coral {
    private DCMotorSim motorSim;

    /** Creates a simulated version of the coral subsystem. */
    public CoralSim() {
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        motorSim.update(0.02);

        TurboLogger.log("/Coral/Percent/Actual", getPercent());
        TurboLogger.log("/Coral/Voltage/Actual", getVoltage().in(Volts));
        TurboLogger.log("/Coral/Current", getCurrent().in(Amps));
    }

    @Override
    public void setPercent(double percent) {
        TurboLogger.log("/Coral/Percent/Setpoint", percent);

        motorSim.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        TurboLogger.log("/Coral/Voltage/Setpoint", voltage.in(Volts));

        motorSim.setInputVoltage(voltage.in(Volts));
    }
}
