package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private Command teleopCommand;

    public Robot() {
        robotContainer = new RobotContainer();

        Logger.addDataReceiver(new NT4Publisher());

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
        }

        if (isSimulation() && Constants.isReplay) {
            Logger.setReplaySource(new WPILOGReader("log.wpilog"));
        }

        Logger.start();

        // Adding adjustable PID values
        AdjustableNumbers.register("kPDrive", "/PIDValues/Drivetrain/kPDrive", Constants.DriveConstants.kPDriveDefault);
        AdjustableNumbers.register("kIDrive", "/PIDValues/Drivetrain/kIDrive", Constants.DriveConstants.kIDriveDefault);
        AdjustableNumbers.register("kDDrive", "/PIDValues/Drivetrain/kDDrive", Constants.DriveConstants.kDDriveDefault);
        AdjustableNumbers.register("kPSteer", "/PIDValues/Drivetrain/kPSteer", Constants.DriveConstants.kPSteerDefault);
        AdjustableNumbers.register("kISteer", "/PIDValues/Drivetrain/kISteer", Constants.DriveConstants.kISteerDefault);
        AdjustableNumbers.register("kDSteer", "/PIDValues/Drivetrain/kDSteer", Constants.DriveConstants.kDSteerDefault);
        AdjustableNumbers.register("kPElev", "/PIDValues/Elevator/kP", Constants.ElevatorConstants.kPDefault);
        AdjustableNumbers.register("kIElev", "/PIDValues/Elevator/kI", Constants.ElevatorConstants.kIDefault);
        AdjustableNumbers.register("kDElev", "/PIDValues/Elevator/kD", Constants.ElevatorConstants.kDDefault);
    }

    public static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }

        return Alliance.Blue;
    }

    /** Runs every tick while the robot is on. */
    @Override
    public void robotPeriodic() {
        // Running the scheduled commands
        CommandScheduler.getInstance().run();

        AdjustableNumbers.updateValues();
    }

    /** Runs once when the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    /** Runs every tick while the robot is in Disabled mode. */
    @Override
    public void disabledPeriodic() {}

    /** Runs once when the robot exits Disabled mode. */
    @Override
    public void disabledExit() {}

    /** Runs once when the robot enters Autonomous mode. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand == null) {
            autonomousCommand = Commands.print("No autonomous command configured.");
        }

        autonomousCommand.schedule();
    }

    /** Runs every tick while the robot is in Autonomous mode. */
    @Override
    public void autonomousPeriodic() {}

    /** Runs once when the robot exits Autonomous mode. */
    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    /** Runs once when the robot enters Teleop mode. */
    @Override
    public void teleopInit() {
        teleopCommand = robotContainer.getTeleopCommand();

        if (teleopCommand == null) {
            teleopCommand = Commands.print("No teleop command configured.");
        }

        teleopCommand.schedule();
    }

    /** Runs every tick while the robot is in Teleop mode. */
    @Override
    public void teleopPeriodic() {}

    /** Runs once when the robot exits Teleop mode. */
    @Override
    public void teleopExit() {
        teleopCommand.cancel();
    }

    /** Runs once when the robot enters Test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** Runs every tick while the robot is in Test mode. */
    @Override
    public void testPeriodic() {}

    /** Runs once when the robot leaves Test mode. */
    @Override
    public void testExit() {}
}
