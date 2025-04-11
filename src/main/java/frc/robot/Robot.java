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
        AdjustableValues.registerNumber("Drive_kP", "/Adjustables/Drive/kP", DriveConstants.kPDriveDefault, "Drive_kP_0", "Drive_kP_1", "Drive_kP_2", "Drive_kP_3");
        AdjustableValues.registerNumber("Drive_kI", "/Adjustables/Drive/kI", DriveConstants.kIDriveDefault, "Drive_kI_0", "Drive_kI_1", "Drive_kI_2", "Drive_kI_3");
        AdjustableValues.registerNumber("Drive_kD", "/Adjustables/Drive/kD", DriveConstants.kDDriveDefault, "Drive_kD_0", "Drive_kD_1", "Drive_kD_2", "Drive_kD_3");
        AdjustableValues.registerNumber("Drive_kS", "/Adjustables/Drive/kS", DriveConstants.kSDriveDefault, "Drive_kS_0", "Drive_kS_1", "Drive_kS_2", "Drive_kS_3");
        AdjustableValues.registerNumber("Drive_kV", "/Adjustables/Drive/kV", DriveConstants.kVDriveDefault, "Drive_kV_0", "Drive_kV_1", "Drive_kV_2", "Drive_kV_3");

        AdjustableValues.registerNumber("Steer_kP", "/Adjustables/Steer/kP", DriveConstants.kPSteerDefault, "Steer_kP_0", "Steer_kP_1", "Steer_kP_2", "Steer_kP_3");
        AdjustableValues.registerNumber("Steer_kI", "/Adjustables/Steer/kI", DriveConstants.kISteerDefault, "Steer_kI_0", "Steer_kI_1", "Steer_kI_2", "Steer_kI_3");
        AdjustableValues.registerNumber("Steer_kD", "/Adjustables/Steer/kD", DriveConstants.kDSteerDefault, "Steer_kD_0", "Steer_kD_1", "Steer_kD_2", "Steer_kD_3");
        AdjustableValues.registerNumber("Steer_kS", "/Adjustables/Steer/kS", DriveConstants.kSSteerDefault, "Steer_kS_0", "Steer_kS_1", "Steer_kS_2", "Steer_kS_3");
        AdjustableValues.registerNumber("Steer_kV", "/Adjustables/Steer/kV", DriveConstants.kVSteerDefault, "Steer_kV_0", "Steer_kV_1", "Steer_kV_2", "Steer_kV_3");

        AdjustableValues.registerNumber("X_kP", "/Adjustables/XController/kP", DriveConstants.kPX);
        AdjustableValues.registerNumber("X_kI", "/Adjustables/XController/kI", DriveConstants.kIX);
        AdjustableValues.registerNumber("X_kD", "/Adjustables/XController/kD", DriveConstants.kDX);

        AdjustableValues.registerNumber("Y_kP", "/Adjustables/YController/kP", DriveConstants.kPY);
        AdjustableValues.registerNumber("Y_kI", "/Adjustables/YController/kI", DriveConstants.kIY);
        AdjustableValues.registerNumber("Y_kD", "/Adjustables/YController/kD", DriveConstants.kDY);

        AdjustableValues.registerNumber("Theta_kP", "/Adjustables/ThetaController/kP", DriveConstants.kPTheta);
        AdjustableValues.registerNumber("Theta_kI", "/Adjustables/ThetaController/kI", DriveConstants.kITheta);
        AdjustableValues.registerNumber("Theta_kD", "/Adjustables/ThetaController/kD", DriveConstants.kDTheta);

        AdjustableValues.registerNumber("Elev_kP", "/Adjustables/Elevator/kP", ElevatorConstants.kPDefault);
        AdjustableValues.registerNumber("Elev_kI", "/Adjustables/Elevator/kI", ElevatorConstants.kIDefault);
        AdjustableValues.registerNumber("Elev_kD", "/Adjustables/Elevator/kD", ElevatorConstants.kDDefault);
        AdjustableValues.registerNumber("Elev_kS", "/Adjustables/Elevator/kS", ElevatorConstants.kSDefault);
        AdjustableValues.registerNumber("Elev_kG", "/Adjustables/Elevator/kG", ElevatorConstants.kGDefault);
        AdjustableValues.registerNumber("Elev_kV", "/Adjustables/Elevator/kV", ElevatorConstants.kVDefault);
        AdjustableValues.registerNumber("Elev_kA", "/Adjustables/Elevator/kA", ElevatorConstants.kADefault);

        AdjustableValues.registerNumber("Climb_kP", "/Adjustables/Climb/Climb_kP", ClimbConstants.kPDefault);
        AdjustableValues.registerNumber("Climb_kI", "/Adjustables/Climb/Climb_kI", ClimbConstants.kIDefault);
        AdjustableValues.registerNumber("Climb_kD", "/Adjustables/Climb/Climb_kD", ClimbConstants.kDDefault);
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