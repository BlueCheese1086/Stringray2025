package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;
import java.util.function.Supplier;

public class OverrideCoral extends Command {
    private Coral coral;
    private Supplier<Double> runCarriagePercent;

    /** Creates a new OverideShoot. */
    public OverrideCoral(Coral coral, Supplier<Double> runCarriagePercent) {
        this.coral = coral;
        this.runCarriagePercent = runCarriagePercent;

        addRequirements(coral);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        coral.setPercent(runCarriagePercent.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
