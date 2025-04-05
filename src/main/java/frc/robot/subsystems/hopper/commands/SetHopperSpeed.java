package frc.robot.subsystems.hopper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetHopperSpeed extends Command {
    private Hopper hopper;
    private Supplier<Double> throttle;
    private Supplier<Double> percentSupplier;

    /**
     * Creates a new SetHopperSpeed command.
     * It sets the percent output of the hopper and sets it back to 0 when the
     * command is cancelled.
     * 
     * @param hopper   The hopper subsystem to control.
     * @param throttle The percent speed to run at.
     * @param percentSupplier The max percent to run at.
     */
    public SetHopperSpeed(Hopper hopper, Supplier<Double> throttle, Supplier<Double> percentSupplier) {
        this.hopper = hopper;
        this.throttle = throttle;
        this.percentSupplier = percentSupplier;

        addRequirements(hopper);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        hopper.setPercent(speed * percentSupplier.get());
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        hopper.setPercent(0);
    }
}