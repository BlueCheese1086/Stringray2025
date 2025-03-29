package frc.robot.subsystems.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gyro.Gyro;
import java.util.function.Supplier;

public class AntiTip extends Command {
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Gyro gyro;
    private Supplier<Boolean> shouldStop;
    private Command waitCommand;

    public AntiTip(Drivetrain drivetrain, Elevator elevator, Gyro gyro, Supplier<Boolean> shouldStop) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        waitCommand = new WaitCommand(Constants.TipTimeout);
        waitCommand.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (gyro.getPitch().gte(Constants.TipThreshold)) {
            elevator.getCurrentCommand().cancel();
            drivetrain.getCurrentCommand().cancel();

            elevator.setPosition(Meters.zero());

            waitCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return shouldStop.get();
    }

    @Override
    public void end(boolean interrupted) {
        waitCommand.cancel();
    }
}
