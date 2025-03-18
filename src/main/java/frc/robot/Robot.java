package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

        // Adding adjustable values
        AdjustableValues.register("Drive_kP", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kP", Constants.DriveConstants.kPDriveDefault);
        AdjustableValues.register("Drive_kI", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kI", Constants.DriveConstants.kIDriveDefault);
        AdjustableValues.register("Drive_kD", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kD", Constants.DriveConstants.kDDriveDefault);
        AdjustableValues.register("Drive_kS", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kS", Constants.DriveConstants.kSDriveDefault);
        AdjustableValues.register("Drive_kV", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kV", Constants.DriveConstants.kVDriveDefault);
        AdjustableValues.register("Drive_kA", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Drive/kA", Constants.DriveConstants.kADriveDefault);

        AdjustableValues.register("Steer_kP", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kP", Constants.DriveConstants.kPSteerDefault);
        AdjustableValues.register("Steer_kI", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kI", Constants.DriveConstants.kISteerDefault);
        AdjustableValues.register("Steer_kD", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kD", Constants.DriveConstants.kDSteerDefault);
        AdjustableValues.register("Steer_kS", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kS", Constants.DriveConstants.kSSteerDefault);
        AdjustableValues.register("Steer_kV", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kV", Constants.DriveConstants.kVSteerDefault);
        AdjustableValues.register("Steer_kA", "/AdvantageKit/RealOutputs/Adjustables/Drivetrain/Steer/kA", Constants.DriveConstants.kASteerDefault);

        AdjustableValues.register("Elev_kP", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kP", Constants.ElevatorConstants.kPDefault);
        AdjustableValues.register("Elev_kI", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kI", Constants.ElevatorConstants.kIDefault);
        AdjustableValues.register("Elev_kD", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kD", Constants.ElevatorConstants.kDDefault);

        AdjustableValues.register("Elev_kS_L1", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kS_L1", Constants.ElevatorConstants.kSDefaults[0]);
        AdjustableValues.register("Elev_kS_L2", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kS_L2", Constants.ElevatorConstants.kSDefaults[1]);
        AdjustableValues.register("Elev_kS_L3", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kS_L3", Constants.ElevatorConstants.kSDefaults[2]);
        
        AdjustableValues.register("Elev_kG_L1", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kG_L1", Constants.ElevatorConstants.kGDefaults[0]);
        AdjustableValues.register("Elev_kG_L2", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kG_L2", Constants.ElevatorConstants.kGDefaults[1]);
        AdjustableValues.register("Elev_kG_L3", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kG_L3", Constants.ElevatorConstants.kGDefaults[2]);
        
        AdjustableValues.register("Elev_kV", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kV", Constants.ElevatorConstants.kVDefault);
        
        AdjustableValues.register("Elev_kA_L1", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kA_L1", Constants.ElevatorConstants.kADefaults[0]);
        AdjustableValues.register("Elev_kA_L2", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kA_L2", Constants.ElevatorConstants.kADefaults[1]);
        AdjustableValues.register("Elev_kA_L3", "/AdvantageKit/RealOutputs/Adjustables/Elevator/kA_L3", Constants.ElevatorConstants.kADefaults[2]);
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
