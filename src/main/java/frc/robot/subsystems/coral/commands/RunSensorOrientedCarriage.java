package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.Coral;

public class RunSensorOrientedCarriage extends Command {
    private Coral carriage;
    private double canandcolorProx;

    public RunSensorOrientedCarriage(Coral carriage) {
        this.carriage = carriage;
        addRequirements(carriage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.canandcolorProx = carriage.getSensorProximity();

        carriage.setPercent(0.4);

        if (canandcolorProx < CoralConstants.proximityThreshold) {
            carriage.setPercent(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        carriage.setPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
