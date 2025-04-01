package frc.robot.subsystems.hopper.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

import java.util.function.Supplier;

public class SetHopperVoltage extends Command {
    private Hopper hopper;
    private Supplier<Double> voltageSupplier;

    /**
     * Creates a new SetHopperVoltage command.
     * It sets the voltage output of the hopper and sets it back to 0 when the command is cancelled.
     * 
     * @param hopper The hopper subsystem to control.
     * @param voltageSupplier The voltage to run at.  It is a supplier so it can be tuned while running the motors.
     */
    public SetHopperVoltage(Hopper hopper, Supplier<Double> voltageSupplier) {
        this.hopper = hopper;
        this.voltageSupplier = voltageSupplier;

        addRequirements(hopper);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        hopper.setVoltage(Volts.of(voltageSupplier.get() * HopperConstants.maxPercent));
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        hopper.setVoltage(Volts.zero());
    }
}