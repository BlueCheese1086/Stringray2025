package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetAlgaePercent extends Command {
    private Algae algae;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetAlgaePercent} command.
     * It sets the percent output of the algae motor and resets it back to 0 when the command is cancelled.
     * 
     * @param algae The {@link Algae} subsystem to control.
     * @param throttle The percent speed to run at.
     */
    public SetAlgaePercent(Algae algae, Supplier<Double> throttle) {
        this.algae = algae;
        this.throttle = throttle;

        addRequirements(algae);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        algae.setPercent(speed * AdjustableValues.getNumber("Algae_Percent"));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        algae.setPercent(0);
    }
}