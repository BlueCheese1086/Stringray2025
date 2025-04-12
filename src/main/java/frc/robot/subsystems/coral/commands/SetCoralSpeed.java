package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.coral.Coral;
import java.util.function.Supplier;

public class SetCoralSpeed extends Command {
    private Coral coral;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetCoralSpeed} command.
     * It sets the percent output of the coral motor and sets it back to 0 when the command is cancelled.
     * 
     * @param coral The {@link coral} subsystem to control.
     * @param throttle The percent speed to run at.
     */
    public SetCoralSpeed(Coral coral, Supplier<Double> throttle) {
        this.coral = coral;
        this.throttle = throttle;

        addRequirements(coral);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        coral.setPercent(speed * AdjustableValues.getNumber("Coral_Percent"));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        coral.setPercent(0);
    }
}