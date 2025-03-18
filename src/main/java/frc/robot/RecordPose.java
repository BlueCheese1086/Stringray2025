package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RecordPose extends Command {
    int x = 0;
    Drivetrain drivetrain;

    public RecordPose(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        x += 1;
        Logger.recordOutput("/RecordedPoses/" + x, drivetrain.getPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
