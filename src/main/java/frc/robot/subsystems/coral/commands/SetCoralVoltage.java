package frc.robot.subsystems.coral.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.Coral;
import java.util.function.Supplier;

public class SetCoralVoltage extends Command {
    private Coral coral;
    private Supplier<Double> voltageSupplier;

    /**
     * Creates a new SetCoralVoltage command.
     * It sets the voltage output of the coral motor and sets it back to 0 when the
     * command is cancelled.
     * 
     * @param coral           The coral subsystem to control.
     * @param voltageSupplier The voltage to run at. It is a supplier so it can be
     *                        tuned while running the motors.
     */
    public SetCoralVoltage(Coral coral, Supplier<Double> voltageSupplier) {
        this.coral = coral;
        this.voltageSupplier = voltageSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        coral.setVoltage(Volts.of(voltageSupplier.get() * CoralConstants.maxPercent));
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        coral.setVoltage(Volts.zero());
    }
}