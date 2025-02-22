package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.PIDValues;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(motor, mass, radius, gearing),
        motor,
        minHeight,
        maxHeight,
        true,
        startingHeight
    );

    private PIDController feedback = new PIDController(PIDValues.kPElev, PIDValues.kIElev, PIDValues.kDElev);
    private double feedforward = 0.0;
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double position = sim.getPositionMeters();

        double voltage = MathUtil.clamp(feedforward + feedback.calculate(position), -12.0, 12.0);
        sim.setInputVoltage(voltage);
        sim.update(0.02);

        inputs.position = position;
        inputs.velocity = sim.getVelocityMetersPerSecond();
        inputs.voltages = new double[] {voltage};
        inputs.currents = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedforward = ffVoltage;
        feedback.setSetpoint(position);
    }
}
