package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.util.TurboLogger;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    public Robot() {
        robotContainer = new RobotContainer();

        // Logger.addDataReceiver(new NT4Publisher());

        // if (isReal()) {
        //     Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
        // }

        // if (isSimulation() && Constants.isReplay) {
        //     Logger.setReplaySource(new WPILOGReader("log.wpilog"));
        // }

        // Logger.start();

        // Adding adjustable values
        TurboLogger.log(
                "/Adjustables/Algae/MaxPercent", AlgaeConstants.maxPercent, "Algae_Percent");

        TurboLogger.log("/Adjustables/AutoAlign/X/kP", DriveConstants.kPX, "X_kP");
        TurboLogger.log("/Adjustables/AutoAlign/X/kI", DriveConstants.kIX, "X_kI");
        TurboLogger.log("/Adjustables/AutoAlign/X/kD", DriveConstants.kDX, "X_kD");

        TurboLogger.log("/Adjustables/AutoAlign/Y/kP", DriveConstants.kPY, "Y_kP");
        TurboLogger.log("/Adjustables/AutoAlign/Y/kI", DriveConstants.kIY, "Y_kI");
        TurboLogger.log("/Adjustables/AutoAlign/Y/kD", DriveConstants.kDY, "Y_kD");

        TurboLogger.log("/Adjustables/AutoAlign/Theta/kP", DriveConstants.kPTheta, "Theta_kP");
        TurboLogger.log("/Adjustables/AutoAlign/Theta/kI", DriveConstants.kITheta, "Theta_kI");
        TurboLogger.log("/Adjustables/AutoAlign/Theta/kD", DriveConstants.kDTheta, "Theta_kD");

        TurboLogger.log("/Adjustables/Climb/kP", ClimbConstants.kPDefault, "Climb_kP");
        TurboLogger.log("/Adjustables/Climb/kI", ClimbConstants.kIDefault, "Climb_kI");
        TurboLogger.log("/Adjustables/Climb/kD", ClimbConstants.kDDefault, "Climb_kD");
        TurboLogger.log(
                "/Adjustables/Climb/MaxPercent", ClimbConstants.maxPercent, "Climb_Percent");

        TurboLogger.log(
                "/Adjustables/Coral/MaxPercent", CoralConstants.maxPercent, "Coral_Percent");

        TurboLogger.log(
                "/Adjustables/Drive/kP",
                DriveConstants.kPDriveDefault,
                "Drive_kP",
                "Drive_kP_0",
                "Drive_kP_1",
                "Drive_kP_2",
                "Drive_kP_3");
        TurboLogger.log(
                "/Adjustables/Drive/kI",
                DriveConstants.kIDriveDefault,
                "Drive_kI",
                "Drive_kI_0",
                "Drive_kI_1",
                "Drive_kI_2",
                "Drive_kI_3");
        TurboLogger.log(
                "/Adjustables/Drive/kD",
                DriveConstants.kDDriveDefault,
                "Drive_kD",
                "Drive_kD_0",
                "Drive_kD_1",
                "Drive_kD_2",
                "Drive_kD_3");
        TurboLogger.log(
                "/Adjustables/Drive/kS",
                DriveConstants.kSDriveDefault,
                "Drive_kS",
                "Drive_kS_0",
                "Drive_kS_1",
                "Drive_kS_2",
                "Drive_kS_3");
        TurboLogger.log(
                "/Adjustables/Drive/kV",
                DriveConstants.kVDriveDefault,
                "Drive_kV",
                "Drive_kV_0",
                "Drive_kV_1",
                "Drive_kV_2",
                "Drive_kV_3");
        TurboLogger.log(
                "/Adjustables/Drive/MaxDriveXPercent",
                DriveConstants.driveXPercent,
                "DriveX_Percent");
        TurboLogger.log(
                "/Adjustables/Drive/MaxDriveYPercent",
                DriveConstants.driveYPercent,
                "DriveY_Percent");

        TurboLogger.log("/Adjustables/Elevator/kP", ElevatorConstants.kPDefault, "Elev_kP");
        TurboLogger.log("/Adjustables/Elevator/kI", ElevatorConstants.kIDefault, "Elev_kI");
        TurboLogger.log("/Adjustables/Elevator/kD", ElevatorConstants.kDDefault, "Elev_kD");
        TurboLogger.log("/Adjustables/Elevator/kS", ElevatorConstants.kSDefault, "Elev_kS");
        TurboLogger.log("/Adjustables/Elevator/kG", ElevatorConstants.kGDefault, "Elev_kG");
        TurboLogger.log("/Adjustables/Elevator/kV", ElevatorConstants.kVDefault, "Elev_kV");
        TurboLogger.log("/Adjustables/Elevator/kA", ElevatorConstants.kADefault, "Elev_kA");
        TurboLogger.log(
                "/Adjustables/Elevator/MaxPercent",
                ElevatorConstants.maxPercent,
                "Elevator_Percent");

        TurboLogger.log(
                "/Adjustables/Hopper/MaxPercent", HopperConstants.maxPercent, "Hopper_Percent");

        TurboLogger.log(
                "/Adjustables/Steer/kP",
                DriveConstants.kPSteerDefault,
                "Steer_kP",
                "Steer_kP_0",
                "Steer_kP_1",
                "Steer_kP_2",
                "Steer_kP_3");
        TurboLogger.log(
                "/Adjustables/Steer/kI",
                DriveConstants.kISteerDefault,
                "Steer_kI",
                "Steer_kI_0",
                "Steer_kI_1",
                "Steer_kI_2",
                "Steer_kI_3");
        TurboLogger.log(
                "/Adjustables/Steer/kD",
                DriveConstants.kDSteerDefault,
                "Steer_kD",
                "Steer_kD_0",
                "Steer_kD_1",
                "Steer_kD_2",
                "Steer_kD_3");
        TurboLogger.log(
                "/Adjustables/Steer/kS",
                DriveConstants.kSSteerDefault,
                "Steer_kS",
                "Steer_kS_0",
                "Steer_kS_1",
                "Steer_kS_2",
                "Steer_kS_3");
        TurboLogger.log(
                "/Adjustables/Steer/kV",
                DriveConstants.kVSteerDefault,
                "Steer_kV",
                "Steer_kV_0",
                "Steer_kV_1",
                "Steer_kV_2",
                "Steer_kV_3");
        TurboLogger.log(
                "/Adjustables/Steer/MaxSteerPercent", DriveConstants.steerPercent, "Steer_Percent");

        // Enabling DataLog recording
        TurboLogger.enableDataLogs("test.wpilog");
    }

    /** Runs every tick while the robot is on. */
    @Override
    public void robotPeriodic() {
        // Running the scheduled commands
        CommandScheduler.getInstance().run();
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
