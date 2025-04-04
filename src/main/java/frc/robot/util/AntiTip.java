package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.gyro.Gyro;

public class AntiTip extends Command {
    private Elevator elevator;
    private Gyro gyro;

    /**
     * Creates a new {@link AntiTip} command.
     * It checks the roll and pitch of the gyroscope and slams the elevator down if it is above some threshold.
     * 
     * @param elevator The {@link Elevator} to control.
     * @param gyro The {@link Gyro} to read.
     */
    public AntiTip(Elevator elevator, Gyro gyro) {
        this.elevator = elevator;
        this.gyro = gyro;
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        if (gyro.getPitch().gte(Constants.TipThreshold) || gyro.getRoll().gte(Constants.TipThreshold)) {
            elevator.setPosition(ElevatorPositions.STOW);
        }
    }
}