package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeSim extends Algae {
    private DCMotorSim algae;

    public AlgaeSim() {
        this.algae = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.02, 1), DCMotor.getKrakenX60(1));
    }

    @Override
    public void periodic() {
        algae.update(0.02);

        SmartDashboard.putNumber("/Algae/Current", getCurrent().in(Amps));
        SmartDashboard.putNumber("/Algae/Percent/Actual", getPercent());
        SmartDashboard.putNumber("/Algae/Voltage/Actual", getVoltage().in(Volts));
    }

    @Override
    public void setPercent(double percent) {
        SmartDashboard.putNumber("/Algae/Percent/Setpoint", percent);

        algae.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        SmartDashboard.putNumber("/Algae/Voltage/Setpoint", voltage.in(Volts));

        algae.setInputVoltage(voltage.in(Volts));
    }

    @Override
    public Current getCurrent() {
        return Amps.of(algae.getCurrentDrawAmps());
    }

    @Override
    public double getPercent() {
        return algae.getInputVoltage() / RobotController.getInputVoltage();
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(algae.getInputVoltage());
    }
}