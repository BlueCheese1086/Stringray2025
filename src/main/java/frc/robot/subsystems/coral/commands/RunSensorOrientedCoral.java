package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.Coral;

public class RunSensorOrientedCoral extends Command {
    private Coral coral;
    private double canandcolorProx;

    public RunSensorOrientedCoral(Coral coral) {
        this.coral = coral;
        addRequirements(coral);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.canandcolorProx = coral.getSensorProximity();

        coral.setPercent(0.4);

        if (canandcolorProx < CoralConstants.proximityThreshold) {
            coral.setPercent(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coral.setPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
