package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.carriage.Carriage;
import java.util.function.Supplier;

public class RunAlgaeRoller extends Command {
    private Algae algae;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new RunAlgaeRoller command.
     * It sets the percent output of the algae roller on the carriage and sets it back to 0 when the command is cancelled.
     * 
     * @param carriage The carriage subsystem to control.
     * @param percentSupplier The percent output to run at.  It is a supplier so it can be tuned while running the motors.
     */
    public RunAlgaeRoller(Algae algae, Supplier<Double> percentSupplier) {
        this.algae = algae;
        this.percentSupplier = percentSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        algae.set(percentSupplier.get());
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        algae.set(0);
    }
}