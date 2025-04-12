package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetAlgaeVoltage extends Command {
    private Algae algae;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetAlgaeVoltage} command.
     * It sets the voltage output of the algae motor and resets it back to 0 when the command is cancelled.
     * 
     * @param algae The {@link Algae} subsystem to control.
     * @param throttle The percent voltage to run at.
     */
    public SetAlgaeVoltage(Algae algae, Supplier<Double> throttle) {
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

        algae.setVoltage(speed * AdjustableValues.getNumber("Algae_Percent") * RobotController.getInputVoltage());
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        algae.setVoltage(0);
    }
}