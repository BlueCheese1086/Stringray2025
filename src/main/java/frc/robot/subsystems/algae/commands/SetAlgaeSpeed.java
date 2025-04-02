package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetAlgaeSpeed extends Command {
    private Algae algae;
    private Supplier<Double> throttle;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new SetAlgaeSpeed command.
     * It sets the percent output of the algae roller and resets it back to 0 when the
     * command is cancelled.
     * 
     * @param algae           The algae subsystem to control.
     * @param throttle        The percent speed to run at.
     * @param percentSupplier The maximum percent output.
     */
    public SetAlgaeSpeed(Algae algae, Supplier<Double> throttle, Supplier<Double> percentSupplier) {
        this.algae = algae;
        this.throttle = throttle;
        this.percentSupplier = percentSupplier;

        addRequirements(algae);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        algae.setPercent(speed * percentSupplier.get());
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