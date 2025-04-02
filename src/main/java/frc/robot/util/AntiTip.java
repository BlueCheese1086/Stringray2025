package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.gyro.Gyro;

public class AntiTip extends Command {
    private Elevator elevator;
    private Gyro gyro;

    public AntiTip(Elevator elevator, Gyro gyro) {
        this.elevator = elevator;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (gyro.getPitch().gte(Constants.TipThreshold) || gyro.getRoll().gte(Constants.TipThreshold)) {
            elevator.setPosition(ElevatorPositions.STOW);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}