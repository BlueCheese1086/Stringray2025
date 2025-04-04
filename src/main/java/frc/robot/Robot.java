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

        AdjustableValues.registerNumber("Elev_kS_L1", "/Adjustables/Elevator/kS_L1", ElevatorConstants.kSDefaults[0]);
        AdjustableValues.registerNumber("Elev_kS_L2", "/Adjustables/Elevator/kS_L2", ElevatorConstants.kSDefaults[1]);
        AdjustableValues.registerNumber("Elev_kS_L3", "/Adjustables/Elevator/kS_L3", ElevatorConstants.kSDefaults[2]);

        AdjustableValues.registerNumber("Elev_kG_L1", "/Adjustables/Elevator/kG_L1", ElevatorConstants.kGDefaults[0]);
        AdjustableValues.registerNumber("Elev_kG_L2", "/Adjustables/Elevator/kG_L2", ElevatorConstants.kGDefaults[1]);
        AdjustableValues.registerNumber("Elev_kG_L3", "/Adjustables/Elevator/kG_L3", ElevatorConstants.kGDefaults[2]);

        AdjustableValues.registerNumber("Elev_kV_L1", "/Adjustables/Elevator/kV_L1", ElevatorConstants.kVDefaults[0]);
        AdjustableValues.registerNumber("Elev_kV_L2", "/Adjustables/Elevator/kV_L2", ElevatorConstants.kVDefaults[1]);
        AdjustableValues.registerNumber("Elev_kV_L3", "/Adjustables/Elevator/kV_L3", ElevatorConstants.kVDefaults[2]);

        AdjustableValues.registerNumber("Elev_kA_L1", "/Adjustables/Elevator/kA_L1", ElevatorConstants.kADefaults[0]);
        AdjustableValues.registerNumber("Elev_kA_L2", "/Adjustables/Elevator/kA_L2", ElevatorConstants.kADefaults[1]);
        AdjustableValues.registerNumber("Elev_kA_L3", "/Adjustables/Elevator/kA_L3", ElevatorConstants.kADefaults[2]);

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