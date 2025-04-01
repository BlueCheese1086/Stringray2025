package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeConstants;
import java.util.function.Supplier;

public class SetAlgaePercent extends Command {
    private Algae algae;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new SetAlgaePercent command.
     * It sets the percent output of the algae roller and sets it back to 0 when the
     * command is cancelled.
     * 
     * @param algae           The algae subsystem to control.
     * @param percentSupplier The percent output to run at. It is a supplier so it
     *                        can be tuned while running the motors.
     */
    public SetAlgaePercent(Algae algae, Supplier<Double> percentSupplier) {
        this.algae = algae;
        this.percentSupplier = percentSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        algae.setPercent(percentSupplier.get() * AlgaeConstants.maxPercent);
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        algae.setPercent(0);
    }
}