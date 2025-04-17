package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Poses;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPositions;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import java.util.function.BooleanSupplier;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private Coral coral;
    private Drive drive;
    private Elevator elevator;
    private Gyro gyro;
    private Vision vision;
    private Climb climb;
    private Algae algae;
    private Hopper hopper;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            gyro = new Gyro(new GyroIOPigeon2(RobotMap.GYRO_Pigeon2Id));

            vision = new Vision(
                    new CameraIOReal(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOReal(VisionConstants.rCameraName, VisionConstants.rCameraTransform));

            drive = new Drive(gyro, vision,
                    new ModuleIOTalonFX(0),
                    new ModuleIOTalonFX(1),
                    new ModuleIOTalonFX(2),
                    new ModuleIOTalonFX(3));

            algae = new Algae(new AlgaeIOReal(RobotMap.ALGAE_MotorId, RobotMap.ALGAE_LaserId));
            
            hopper = new Hopper(new HopperIOReal(RobotMap.HOPPER_MotorId, RobotMap.HOPPER_LaserId));
            
            coral = new Coral(new CoralIOReal(RobotMap.CORAL_MotorId, RobotMap.CORAL_SensorId, RobotMap.CORAL_LaserId));
            
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
            
            climb = new Climb(new ClimbIOReal(RobotMap.CLIMB_MotorId));
        } else {
            // Reminder that this does nothing.
            gyro = new Gyro(new GyroIOSim());

            vision = new Vision(
                    new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
            
            drive = new Drive(gyro, vision,
                    new ModuleIOSim(0),
                    new ModuleIOSim(1),
                    new ModuleIOSim(2),
                    new ModuleIOSim(3));

            coral = new Coral(new CoralIOSim());
            
            hopper = new Hopper(new HopperIOSim());
            
            elevator = new Elevator(new ElevatorIOSim());
            
            algae = new Algae(new AlgaeIOSim());
            
            climb = new Climb(new ClimbIOSim());
        }

        // Anti-Tip command
        // This doesn't work in sim due to the sim gyro not actually doing anything
        new AntiTip(elevator::setPosition, gyro::getPitch, gyro::getRoll).schedule();

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {
        // Override condition used for many of the commands.
        // Defining it once rather than 15 times.
        BooleanSupplier joystickOverride = () -> (
            !MathUtils.withinDeadband(driverController.getLeftX(), Constants.deadband) ||
            !MathUtils.withinDeadband(driverController.getLeftY(), Constants.deadband) ||
            !MathUtils.withinDeadband(driverController.getRightX(), Constants.deadband) ||
            !MathUtils.withinDeadband(driverController.getRightY(), Constants.deadband));

        // Driver Controls

        // Normal drive
        drive.setDefaultCommand(
            DriveCommands.drive(
                    drive,
                    driverController::getLeftY,
                    driverController::getLeftX,
                    driverController::getRightX,
                    () -> false));

        // Precision Mode
        driverController.leftBumper().or(driverController.rightBumper())
            .whileTrue(
                DriveCommands.drive(
                        drive,
                        () -> driverController.getLeftY() * DriveConstants.precisionPercent,
                        () -> driverController.getLeftX() * DriveConstants.precisionPercent,
                        () -> driverController.getRightX() * DriveConstants.precisionPercent,
                        () -> false));
        // Precision Mode
        // It limits the max speeds through the AdjustableValues class and puts them back to their previous percents when done. 
        // double[] percents = new double[3];
        // driverController.leftBumper().whileTrue(Commands.runEnd(
        //     () -> {
        //         percents[0] = AdjustableValues.getNumber("DriveX_Percent");
        //         percents[1] = AdjustableValues.getNumber("DriveY_Percent");
        //         percents[2] = AdjustableValues.getNumber("Steer_Percent");

        //         AdjustableValues.setNumber("DriveX_Percent", DriveConstants.precisionPercent);
        //         AdjustableValues.setNumber("DriveY_Percent", DriveConstants.precisionPercent);
        //         AdjustableValues.setNumber("Steer_Percent", DriveConstants.precisionPercent);
        //     },
        //     () -> {
        //         AdjustableValues.setNumber("DriveX_Percent", percents[0]);
        //         AdjustableValues.setNumber("DriveY_Percent", percents[1]);
        //         AdjustableValues.setNumber("Steer_Percent", percents[2]);
        //     }));

        // Reset gyro
        driverController.b().onTrue(Commands.runOnce(gyro::reset));

        // Toggle X State
        driverController.x().toggleOnTrue(DriveCommands.xStates(drive).until(joystickOverride));

        // Log current robot pose
        driverController.y().onTrue(new RecordPose(drive::getPose));

        // Pathfinding controls
        // Override pathfinding by moving any joystick or by pressing button again.

        // Pathfind to left side of reef
        driverController.back()
            .toggleOnTrue(DriveCommands.pathfindToNearestPose(drive, Poses.REEF_Left)
            .until(joystickOverride));

        // Pathfind to right side of reef.
        driverController.start()
            .toggleOnTrue(DriveCommands.pathfindToNearestPose(drive, Poses.REEF_Right)
            .until(joystickOverride));

        // Intake Coral & Algae
        driverController.leftTrigger(Constants.deadband)
            .whileTrue(CoralCommands.setSpeed(coral, driverController::getLeftTriggerAxis))
            .whileTrue(AlgaeCommands.setPercent(algae, driverController::getLeftTriggerAxis))
            .whileTrue(HopperCommands.setPercent(hopper, driverController::getLeftTriggerAxis));
        
        // Outtake Coral & Algae
        driverController.rightTrigger(Constants.deadband)
            .whileTrue(CoralCommands.setSpeed(coral, driverController::getRightTriggerAxis))
            .whileTrue(AlgaeCommands.setPercent(algae, driverController::getRightTriggerAxis))
            .whileTrue(HopperCommands.setPercent(hopper, driverController::getRightTriggerAxis));

        // Operator Controls

        // Reset Elevator Encoder
        operatorController.start().onTrue(Commands.run(elevator::resetEncoder).ignoringDisable(true));
        operatorController.back().onTrue(Commands.run(elevator::resetEncoder).ignoringDisable(true));

        // Set Elevator Heights
        operatorController.leftBumper().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.STOW));
        operatorController.leftTrigger(0.2).onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.STOW));
        operatorController.b().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L2));
        operatorController.a().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L1));
        operatorController.x().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L3));
        operatorController.y().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L4));
        operatorController.rightBumper().onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L3Algae));
        operatorController.rightTrigger(0.2).onTrue(ElevatorCommands.setHeight(elevator, ElevatorPositions.L2Algae));
        
        // Elevator manual controls
        operatorController.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, Constants.deadband)
            .whileTrue(ElevatorCommands.setVoltage(elevator, operatorController::getRightY));

        // Set Climb Positions
        operatorController.povLeft().onTrue(ClimbCommands.setAngle(climb, ClimbPositions.GRAB));
        operatorController.povRight().onTrue(ClimbCommands.setAngle(climb, ClimbPositions.HANG));
        operatorController.povDown().onTrue(ClimbCommands.setAngle(climb, ClimbPositions.STOW));

        // Climb manual controls
        operatorController.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, Constants.deadband)
            .whileTrue(ClimbCommands.setVoltage(climb, operatorController::getLeftY));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }
}