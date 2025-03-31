package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.Coral;
import java.util.function.Supplier;

public class SetCoralPercent extends Command {
    private Coral coral;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new SetCoralPercent command.
     * It sets the percent output of the coral motor and sets it back to 0 when the command is cancelled.
     * 
     * @param coral The coral subsystem to control.
     * @param percentSupplier The percent output to run at.  It is a supplier so it can be tuned while running the motors.
     */
    public SetCoralPercent(Coral coral, Supplier<Double> percentSupplier) {
        this.coral = coral;
        this.percentSupplier = percentSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        coral.setPercent(percentSupplier.get());
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return coral.getCanandColorProximity() > CoralConstants.proximityThreshold;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        coral.setPercent(0);
    }
}