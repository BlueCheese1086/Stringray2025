package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ResetPosition extends Command {
    private Elevator elevator;

    /**
     * Resets the elevator's position by zeroing the encoders
     * 
     * @param elevator The elevator subsystem to use.
     */
    public ResetPosition(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}