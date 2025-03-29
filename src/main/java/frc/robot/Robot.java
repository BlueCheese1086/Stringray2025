package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

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
        }Logger.start();

        // Adding adjustable values
        AdjustableValues.registerNumber("Drive_kP", "/Adjustables/Drivetrain/Drive_kP", DriveConstants.kPDriveDefault, "Drive_kP_0", "Drive_kP_1", "Drive_kP_2", "Drive_kP_3");
        AdjustableValues.registerNumber("Drive_kI", "/Adjustables/Drivetrain/Drive_kI", DriveConstants.kIDriveDefault, "Drive_kI_0", "Drive_kI_1", "Drive_kI_2", "Drive_kI_3");
        AdjustableValues.registerNumber("Drive_kD", "/Adjustables/Drivetrain/Drive_kD", DriveConstants.kDDriveDefault, "Drive_kD_0", "Drive_kD_1", "Drive_kD_2", "Drive_kD_3");
        AdjustableValues.registerNumber("Drive_kS", "/Adjustables/Drivetrain/Drive_kS", DriveConstants.kSDriveDefault, "Drive_kS_0", "Drive_kS_1", "Drive_kS_2", "Drive_kS_3");
        AdjustableValues.registerNumber("Drive_kV", "/Adjustables/Drivetrain/Drive_kV", DriveConstants.kVDriveDefault, "Drive_kV_0", "Drive_kV_1", "Drive_kV_2", "Drive_kV_3");
        AdjustableValues.registerNumber("Drive_kA", "/Adjustables/Drivetrain/Drive_kA", DriveConstants.kADriveDefault, "Drive_kA_0", "Drive_kA_1", "Drive_kA_2", "Drive_kA_3");

        AdjustableValues.registerNumber("Steer_kP", "/Adjustables/Drivetrain/Steer_kP", DriveConstants.kPSteerDefault, "Steer_kP_0", "Steer_kP_1", "Steer_kP_2", "Steer_kP_3");
        AdjustableValues.registerNumber("Steer_kI", "/Adjustables/Drivetrain/Steer_kI", DriveConstants.kISteerDefault, "Steer_kI_0", "Steer_kI_1", "Steer_kI_2", "Steer_kI_3");
        AdjustableValues.registerNumber("Steer_kD", "/Adjustables/Drivetrain/Steer_kD", DriveConstants.kDSteerDefault, "Steer_kD_0", "Steer_kD_1", "Steer_kD_2", "Steer_kD_3");
        AdjustableValues.registerNumber("Steer_kS", "/Adjustables/Drivetrain/Steer_kS", DriveConstants.kSSteerDefault, "Steer_kS_0", "Steer_kS_1", "Steer_kS_2", "Steer_kS_3");
        AdjustableValues.registerNumber("Steer_kV", "/Adjustables/Drivetrain/Steer_kV", DriveConstants.kVSteerDefault, "Steer_kV_0", "Steer_kV_1", "Steer_kV_2", "Steer_kV_3");
        AdjustableValues.registerNumber("Steer_kA", "/Adjustables/Drivetrain/Steer_kA", DriveConstants.kASteerDefault, "Steer_kA_0", "Steer_kA_1", "Steer_kA_2", "Steer_kA_3");

        AdjustableValues.registerNumber("Elev_kP", "/Adjustables/Elevator/kP", ElevatorConstants.kPDefault);
        AdjustableValues.registerNumber("Elev_kI", "/Adjustables/Elevator/kI", ElevatorConstants.kIDefault);
        AdjustableValues.registerNumber("Elev_kD", "/Adjustables/Elevator/kD", ElevatorConstants.kDDefault);

        AdjustableValues.registerNumber("Elev_kS_L1", "/Adjustables/Elevator/kS_L1", ElevatorConstants.kSDefaults[0]);
        AdjustableValues.registerNumber("Elev_kS_L2", "/Adjustables/Elevator/kS_L2", ElevatorConstants.kSDefaults[1]);
        AdjustableValues.registerNumber("Elev_kS_L3", "/Adjustables/Elevator/kS_L3", ElevatorConstants.kSDefaults[2]);
        
        AdjustableValues.registerNumber("Elev_kG_L1", "/Adjustables/Elevator/kG_L1", ElevatorConstants.kGDefaults[0]);
        AdjustableValues.registerNumber("Elev_kG_L2", "/Adjustables/Elevator/kG_L2", ElevatorConstants.kGDefaults[1]);
        AdjustableValues.registerNumber("Elev_kG_L3", "/Adjustables/Elevator/kG_L3", ElevatorConstants.kGDefaults[2]);
        
        AdjustableValues.registerNumber("Elev_kV", "/Adjustables/Elevator/kV", ElevatorConstants.kVDefault);
        
        AdjustableValues.registerNumber("Elev_kA_L1", "/Adjustables/Elevator/kA_L1", ElevatorConstants.kADefaults[0]);
        AdjustableValues.registerNumber("Elev_kA_L2", "/Adjustables/Elevator/kA_L2", ElevatorConstants.kADefaults[1]);
        AdjustableValues.registerNumber("Elev_kA_L3", "/Adjustables/Elevator/kA_L3", ElevatorConstants.kADefaults[2]);

        AdjustableValues.registerNumber("X_kP", "/Adjustables/Drivetrain/XController_kP", DriveConstants.kPXControllerDefault);
        AdjustableValues.registerNumber("X_kI", "/Adjustables/Drivetrain/XController_kI", DriveConstants.kIXControllerDefault);
        AdjustableValues.registerNumber("X_kD", "/Adjustables/Drivetrain/XController_kD", DriveConstants.kDXControllerDefault);

        AdjustableValues.registerNumber("Y_kP", "/Adjustables/Drivetrain/YController_kP", DriveConstants.kPYControllerDefault);
        AdjustableValues.registerNumber("Y_kI", "/Adjustables/Drivetrain/YController_kI", DriveConstants.kIYControllerDefault);
        AdjustableValues.registerNumber("Y_kD", "/Adjustables/Drivetrain/YController_kD", DriveConstants.kDYControllerDefault);

        AdjustableValues.registerNumber("Theta_kP", "/Adjustables/Drivetrain/ThetaController_kP", DriveConstants.kPThetaControllerDefault);
        AdjustableValues.registerNumber("Theta_kI", "/Adjustables/Drivetrain/ThetaController_kI", DriveConstants.kIThetaControllerDefault);
        AdjustableValues.registerNumber("Theta_kD", "/Adjustables/Drivetrain/ThetaController_kD", DriveConstants.kDThetaControllerDefault);
    }

    /** Runs every tick while the robot is on. */
    @Override
    public void robotPeriodic() {
        // Running the scheduled commands
        CommandScheduler.getInstance().run();

        robotContainer.periodic();
        
        AdjustableValues.updateValues();
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
    public void teleopInit() {}

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
