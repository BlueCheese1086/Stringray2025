package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.Coral;
import java.util.function.Supplier;

public class IntakeWithSensor extends Command {
    private Coral coral;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new {@link IntakeWithSensor} command.
     * It runs the coral motor at some speed and cancels itself when the CANandColor sensor detects an object.
     * 
     * @param coral The {@link Coral} subsystem to control.
     * @param percentSupplier The percent speed to run at.
     */
    public IntakeWithSensor(Coral coral, Supplier<Double> percentSupplier) {
        this.coral = coral;
        this.percentSupplier = percentSupplier;

        addRequirements(coral);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        if (coral.getSensorProximity() < CoralConstants.proximityThreshold) cancel();

        coral.setPercent(percentSupplier.get());
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        coral.setPercent(0);
    }
}