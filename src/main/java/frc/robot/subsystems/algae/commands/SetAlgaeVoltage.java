package frc.robot.subsystems.algae.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeConstants;
import java.util.function.Supplier;

public class SetAlgaeVoltage extends Command {
    private Algae algae;
    private Supplier<Double> voltageSupplier;

    /**
     * Creates a new SetAlgaeVoltage command.
     * It sets the voltage output of the algae roller and sets it back to 0 when the
     * command is cancelled.
     * 
     * @param algae           The algae subsystem to control.
     * @param voltageSupplier The voltage to run at. It is a supplier so it can be
     *                        tuned while running the motors.
     */
    public SetAlgaeVoltage(Algae algae, Supplier<Double> voltageSupplier) {
        this.algae = algae;
        this.voltageSupplier = voltageSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        algae.setVoltage(Volts.of(voltageSupplier.get() * AlgaeConstants.maxPercent));
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        algae.setVoltage(Volts.zero());
    }
}