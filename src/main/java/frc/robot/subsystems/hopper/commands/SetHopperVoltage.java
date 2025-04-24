
package frc.robot.subsystems.hopper.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SetHopperVoltage extends Command {
    private Hopper hopper;
    private Supplier<Double> throttle;

    /**
     * Creates a new {@link SetHopperVoltage} command.
     * It sets the voltage output of the hopper and sets it back to 0 when the command is cancelled.
     *
     * @param hopper The {@link Hopper} subsystem to control.
     * @param throttle The percent of max voltage to run at.
     */
    public SetHopperVoltage(Hopper hopper, Supplier<Double> throttle) {
        this.hopper = hopper;
        this.throttle = throttle;

        addRequirements(hopper);
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        double speed = throttle.get();

        speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
        speed = Math.copySign(speed * speed, speed);

        hopper.setVoltage(Volts.of(speed));// * AdjustableValues.getNumber("Hopper_Percent") * RobotController.getInputVoltage()));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        hopper.setVoltage(Volts.zero());
    }
}
