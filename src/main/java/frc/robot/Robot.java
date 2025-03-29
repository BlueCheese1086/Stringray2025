package frc.robot;

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

        // Adding adjustable values
        AdjustableValues.registerNumber("Drive_kP", "/Adjustables/Drivetrain/Drive_kP", Constants.DriveConstants.kPDriveDefault, "Drive_kP_0", "Drive_kP_1", "Drive_kP_2", "Drive_kP_3");
        AdjustableValues.registerNumber("Drive_kI", "/Adjustables/Drivetrain/Drive_kI", Constants.DriveConstants.kIDriveDefault, "Drive_kI_0", "Drive_kI_1", "Drive_kI_2", "Drive_kI_3");
        AdjustableValues.registerNumber("Drive_kD", "/Adjustables/Drivetrain/Drive_kD", Constants.DriveConstants.kDDriveDefault, "Drive_kD_0", "Drive_kD_1", "Drive_kD_2", "Drive_kD_3");
        AdjustableValues.registerNumber("Drive_kS", "/Adjustables/Drivetrain/Drive_kS", Constants.DriveConstants.kSDriveDefault, "Drive_kS_0", "Drive_kS_1", "Drive_kS_2", "Drive_kS_3");
        AdjustableValues.registerNumber("Drive_kV", "/Adjustables/Drivetrain/Drive_kV", Constants.DriveConstants.kVDriveDefault, "Drive_kV_0", "Drive_kV_1", "Drive_kV_2", "Drive_kV_3");
        AdjustableValues.registerNumber("Drive_kA", "/Adjustables/Drivetrain/Drive_kA", Constants.DriveConstants.kADriveDefault, "Drive_kA_0", "Drive_kA_1", "Drive_kA_2", "Drive_kA_3");

        AdjustableValues.registerNumber("Steer_kP", "/Adjustables/Drivetrain/Steer_kP", Constants.DriveConstants.kPSteerDefault, "Steer_kP_0", "Steer_kP_1", "Steer_kP_2", "Steer_kP_3");
        AdjustableValues.registerNumber("Steer_kI", "/Adjustables/Drivetrain/Steer_kI", Constants.DriveConstants.kISteerDefault, "Steer_kI_0", "Steer_kI_1", "Steer_kI_2", "Steer_kI_3");
        AdjustableValues.registerNumber("Steer_kD", "/Adjustables/Drivetrain/Steer_kD", Constants.DriveConstants.kDSteerDefault, "Steer_kD_0", "Steer_kD_1", "Steer_kD_2", "Steer_kD_3");
        AdjustableValues.registerNumber("Steer_kS", "/Adjustables/Drivetrain/Steer_kS", Constants.DriveConstants.kSSteerDefault, "Steer_kS_0", "Steer_kS_1", "Steer_kS_2", "Steer_kS_3");
        AdjustableValues.registerNumber("Steer_kV", "/Adjustables/Drivetrain/Steer_kV", Constants.DriveConstants.kVSteerDefault, "Steer_kV_0", "Steer_kV_1", "Steer_kV_2", "Steer_kV_3");
        AdjustableValues.registerNumber("Steer_kA", "/Adjustables/Drivetrain/Steer_kA", Constants.DriveConstants.kASteerDefault, "Steer_kA_0", "Steer_kA_1", "Steer_kA_2", "Steer_kA_3");

        AdjustableValues.registerNumber("Elev_kP", "/Adjustables/Elevator/kP", Constants.ElevatorConstants.kPDefault);
        AdjustableValues.registerNumber("Elev_kI", "/Adjustables/Elevator/kI", Constants.ElevatorConstants.kIDefault);
        AdjustableValues.registerNumber("Elev_kD", "/Adjustables/Elevator/kD", Constants.ElevatorConstants.kDDefault);

        AdjustableValues.registerNumber("Elev_kS_L1", "/Adjustables/Elevator/kS_L1", Constants.ElevatorConstants.kSDefaults[0]);
        AdjustableValues.registerNumber("Elev_kS_L2", "/Adjustables/Elevator/kS_L2", Constants.ElevatorConstants.kSDefaults[1]);
        AdjustableValues.registerNumber("Elev_kS_L3", "/Adjustables/Elevator/kS_L3", Constants.ElevatorConstants.kSDefaults[2]);
        
        AdjustableValues.registerNumber("Elev_kG_L1", "/Adjustables/Elevator/kG_L1", Constants.ElevatorConstants.kGDefaults[0]);
        AdjustableValues.registerNumber("Elev_kG_L2", "/Adjustables/Elevator/kG_L2", Constants.ElevatorConstants.kGDefaults[1]);
        AdjustableValues.registerNumber("Elev_kG_L3", "/Adjustables/Elevator/kG_L3", Constants.ElevatorConstants.kGDefaults[2]);
        
        AdjustableValues.registerNumber("Elev_kV", "/Adjustables/Elevator/kV", Constants.ElevatorConstants.kVDefault);
        
        AdjustableValues.registerNumber("Elev_kA_L1", "/Adjustables/Elevator/kA_L1", Constants.ElevatorConstants.kADefaults[0]);
        AdjustableValues.registerNumber("Elev_kA_L2", "/Adjustables/Elevator/kA_L2", Constants.ElevatorConstants.kADefaults[1]);
        AdjustableValues.registerNumber("Elev_kA_L3", "/Adjustables/Elevator/kA_L3", Constants.ElevatorConstants.kADefaults[2]);

        AdjustableValues.registerNumber("X_kP", "/Adjustables/Drivetrain/XController_kP", Constants.DriveConstants.kPXControllerDefault);
        AdjustableValues.registerNumber("X_kI", "/Adjustables/Drivetrain/XController_kI", Constants.DriveConstants.kIXControllerDefault);
        AdjustableValues.registerNumber("X_kD", "/Adjustables/Drivetrain/XController_kD", Constants.DriveConstants.kDXControllerDefault);

        AdjustableValues.registerNumber("Y_kP", "/Adjustables/Drivetrain/YController_kP", Constants.DriveConstants.kPYControllerDefault);
        AdjustableValues.registerNumber("Y_kI", "/Adjustables/Drivetrain/YController_kI", Constants.DriveConstants.kIYControllerDefault);
        AdjustableValues.registerNumber("Y_kD", "/Adjustables/Drivetrain/YController_kD", Constants.DriveConstants.kDYControllerDefault);

        AdjustableValues.registerNumber("Theta_kP", "/Adjustables/Drivetrain/ThetaController_kP", Constants.DriveConstants.kPThetaControllerDefault);
        AdjustableValues.registerNumber("Theta_kI", "/Adjustables/Drivetrain/ThetaController_kI", Constants.DriveConstants.kIThetaControllerDefault);
        AdjustableValues.registerNumber("Theta_kD", "/Adjustables/Drivetrain/ThetaController_kD", Constants.DriveConstants.kDThetaControllerDefault);


        AdjustableValues.registerNumber("Climb_kP", "/Adjustables/Drivetrain/Climb_kP", Constants.ClimbConstants.kPDefault);
        AdjustableValues.registerNumber("Climb_kI", "/Adjustables/Drivetrain/Climb_kI", Constants.ClimbConstants.kIDefault);
        AdjustableValues.registerNumber("Climb_kD", "/Adjustables/Drivetrain/Climb_kD", Constants.ClimbConstants.kDDefault);
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
