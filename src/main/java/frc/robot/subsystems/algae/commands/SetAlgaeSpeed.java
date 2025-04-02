package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetAlgaeSpeed extends Command {
    private Algae algae;
    private Supplier<Double> throttle;

    /**
     * Creates a new SetAlgaeSpeed command.
     * It sets the percent output of the algae roller and sets it back to 0 when the
     * command is cancelled.
     * 
     * @param algae           The algae subsystem to control.
     * @param throttle        The percent speed to run at.
     */
    public SetAlgaeSpeed(Algae algae, Supplier<Double> throttle) {
        this.algae = algae;
        this.throttle = throttle;

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

        algae.setPercent(speed * AlgaeConstants.maxPercent);
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