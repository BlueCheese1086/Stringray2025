package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.AdjustableValues;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

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

        // Adding adjustable values
        AdjustableValues.registerNumber("X_kP", "/Adjustables/Autos/X_kP", DriveConstants.kPX);
        AdjustableValues.registerNumber("X_kI", "/Adjustables/Autos/X_kI", DriveConstants.kIX);
        AdjustableValues.registerNumber("X_kD", "/Adjustables/Autos/X_kD", DriveConstants.kDX);

        AdjustableValues.registerNumber("Y_kP", "/Adjustables/Autos/Y_kP", DriveConstants.kPY);
        AdjustableValues.registerNumber("Y_kI", "/Adjustables/Autos/Y_kI", DriveConstants.kIY);
        AdjustableValues.registerNumber("Y_kD", "/Adjustables/Autos/Y_kD", DriveConstants.kDY);

        AdjustableValues.registerNumber("Theta_kP", "/Adjustables/Autos/Theta_kP", DriveConstants.kPTheta);
        AdjustableValues.registerNumber("Theta_kI", "/Adjustables/Autos/Theta_kI", DriveConstants.kITheta);
        AdjustableValues.registerNumber("Theta_kD", "/Adjustables/Autos/Theta_kD", DriveConstants.kDTheta);

        AdjustableValues.registerNumber("Elev_kP", "/Adjustables/Elevator/kP", ElevatorConstants.kPDefault);
        AdjustableValues.registerNumber("Elev_kI", "/Adjustables/Elevator/kI", ElevatorConstants.kIDefault);
        AdjustableValues.registerNumber("Elev_kD", "/Adjustables/Elevator/kD", ElevatorConstants.kDDefault);

        AdjustableValues.registerNumber("Elev_L1_kS", "/Adjustables/Elevator/L1_kS", ElevatorConstants.kSDefaults[0]);
        AdjustableValues.registerNumber("Elev_L1_kG", "/Adjustables/Elevator/L1_kG", ElevatorConstants.kGDefaults[0]);
        AdjustableValues.registerNumber("Elev_L1_kV", "/Adjustables/Elevator/L1_kV", ElevatorConstants.kVDefaults[0]);
        AdjustableValues.registerNumber("Elev_L1_kA", "/Adjustables/Elevator/L1_kA", ElevatorConstants.kADefaults[0]);

        AdjustableValues.registerNumber("Elev_L2_kS", "/Adjustables/Elevator/L2_kS", ElevatorConstants.kSDefaults[1]);
        AdjustableValues.registerNumber("Elev_L2_kG", "/Adjustables/Elevator/L2_kG", ElevatorConstants.kGDefaults[1]);
        AdjustableValues.registerNumber("Elev_L2_kV", "/Adjustables/Elevator/L2_kV", ElevatorConstants.kVDefaults[1]);
        AdjustableValues.registerNumber("Elev_L2_kA", "/Adjustables/Elevator/L2_kA", ElevatorConstants.kADefaults[1]);

        AdjustableValues.registerNumber("Elev_L3_kS", "/Adjustables/Elevator/L3_kS", ElevatorConstants.kSDefaults[2]);
        AdjustableValues.registerNumber("Elev_L3_kG", "/Adjustables/Elevator/L3_kG", ElevatorConstants.kGDefaults[2]);
        AdjustableValues.registerNumber("Elev_L3_kV", "/Adjustables/Elevator/L3_kV", ElevatorConstants.kVDefaults[2]);
        AdjustableValues.registerNumber("Elev_L3_kA", "/Adjustables/Elevator/L3_kA", ElevatorConstants.kADefaults[2]);

        AdjustableValues.registerNumber("Climb_kP", "/Adjustables/Drivetrain/Climb_kP", ClimbConstants.kPDefault);
        AdjustableValues.registerNumber("Climb_kI", "/Adjustables/Drivetrain/Climb_kI", ClimbConstants.kIDefault);
        AdjustableValues.registerNumber("Climb_kD", "/Adjustables/Drivetrain/Climb_kD", ClimbConstants.kDDefault);
    }

    /** Runs every tick while the robot is on. */
    @Override
    public void robotPeriodic() {
        // Running the scheduled commands
        CommandScheduler.getInstance().run();

        AdjustableValues.updateValues();
    }

    /** Runs once when the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    /** Runs every tick while the robot is in Disabled mode. */
    @Override
    public void disabledPeriodic() {}

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
    public void teleopInit() {}

    /** Runs every tick while the robot is in Teleop mode. */
    @Override
    public void teleopPeriodic() {}

    /** Runs once when the robot enters Test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** Runs every tick while the robot is in Test mode. */
    @Override
    public void testPeriodic() {}
}