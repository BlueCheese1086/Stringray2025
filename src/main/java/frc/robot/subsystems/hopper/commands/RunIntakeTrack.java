package frc.robot.subsystems.hopper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;

import java.util.function.Supplier;

public class RunIntakeTrack extends Command {
    private Hopper hopper;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new RunIntakeTrack command.
     * It sets the percent output of the track on the intake and sets it back to 0 when the command is cancelled.
     * 
     * @param carriage The carriage subsystem to control.
     * @param percentSupplier The percent output to run at.  It is a supplier so it can be tuned while running the motors.
     */
    public RunIntakeTrack(Hopper hopper, Supplier<Double> percentSupplier) {
        this.hopper = hopper;
        this.percentSupplier = percentSupplier;

        addRequirements(hopper);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        hopper.setTrackPercent(percentSupplier.get());
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        hopper.setTrackPercent(0);
    }
}