package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Poses;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.algae.commands.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPositions;
import frc.robot.subsystems.climb.commands.*;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.coral.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.elevator.commands.*;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.hopper.commands.*;
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
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));

            algae = new Algae(new AlgaeIOReal(RobotMap.ALGAE_MotorId, RobotMap.ALGAE_LaserId));
            
            hopper = new Hopper(new HopperIOReal(RobotMap.HOPPER_MotorId, RobotMap.HOPPER_LaserId));
            
            coral = new Coral(new CoralIOReal(RobotMap.CORAL_MotorId, RobotMap.CORAL_SensorId, RobotMap.CORAL_LaserId));
            
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
            
            climb = new Climb(new ClimbIOReal(RobotMap.CLIMB_MotorId));
        } else {
            vision = new Vision(
                    new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));

            // Reminder that this does nothing.
            gyro = new Gyro(new GyroIOSim());
            
            drive = new Drive(gyro, vision,
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));

            coral = new Coral(new CoralIOSim());
            
            hopper = new Hopper(new HopperIOSim());
            
            elevator = new Elevator(new ElevatorIOSim());
            
            algae = new Algae(new AlgaeIOSim());
            
            climb = new Climb(new ClimbIOSim());
        }

        // Anti-Tip command
        if (RobotBase.isReal()) {
            new AntiTip(elevator::setPosition, gyro::getPitch, gyro::getRoll).schedule();
        }

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
                DriveCommands.joystickDrive(
                        drive,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX,
                        () -> 1.0));

        // Precision Mode
        driverController.leftBumper().or(driverController.rightBumper())
            .whileTrue(
                DriveCommands.joystickDrive(
                        drive,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX,
                        () -> 0.2));        

        // Reset gyro (Only works IRL)
        if (Robot.isReal()) {
            driverController.b().onTrue(Commands.runOnce(gyro::reset));
        }

        // Toggle X State
        driverController.x().toggleOnTrue(DriveCommands.xStates(drive).until(joystickOverride));

        // Log current robot pose
        driverController.y().onTrue(new RecordPose(drive::getPose));

        // Pathfinding controls
        // Override pathfinding by moving any joystick or by pressing button again.

        // Pathfind to left side of reef
        driverController.back()
            .toggleOnTrue(DriveCommands.pidDriveToPose(drive, drive.getPose().nearest(Poses.REEF_Left))
            .until(joystickOverride));

        // Pathfind to right side of reef.
        driverController.start()
            .toggleOnTrue(DriveCommands.pidDriveToPose(drive, drive.getPose().nearest(Poses.REEF_Right))
            .until(joystickOverride));

        // Intake Coral & Algae
        driverController.leftTrigger(Constants.deadband)
            .whileTrue(new SetCoralSpeed(coral, driverController::getLeftTriggerAxis, () -> 1.0))
            .whileTrue(new SetAlgaeSpeed(algae, driverController::getLeftTriggerAxis, () -> 1.0))
            .whileTrue(new SetHopperSpeed(hopper, driverController::getLeftTriggerAxis, () -> 1.0));
        
        // Outtake Coral & Algae
        driverController.rightTrigger(Constants.deadband)
            .whileTrue(new SetCoralSpeed(coral, driverController::getRightTriggerAxis, () -> 1.0))
            .whileTrue(new SetAlgaeSpeed(algae, driverController::getRightTriggerAxis, () -> 1.0))
            .whileTrue(new SetHopperSpeed(hopper, driverController::getRightTriggerAxis, () -> 1.0));


        // Operator Controls

        // Reset Elevator Encoder
        operatorController.start().onTrue(Commands.run(() -> elevator.resetEncoder()).ignoringDisable(true));
        operatorController.back().onTrue(Commands.run(() -> elevator.resetEncoder()).ignoringDisable(true));

        // Set Elevator Heights
        operatorController.leftBumper().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.STOW));
        operatorController.leftTrigger(0.2).onTrue(new SetElevatorHeight(elevator, ElevatorPositions.STOW));
        operatorController.a().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L1));
        operatorController.b().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L2));
        operatorController.x().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L3));
        operatorController.y().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L4));
        operatorController.rightBumper().onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L3Algae));
        operatorController.rightTrigger(0.2).onTrue(new SetElevatorHeight(elevator, ElevatorPositions.L2Algae));
        
        // Elevator manual controls
        new SetElevatorSpeed(elevator, () -> operatorController.getRightY() * 0.5, () -> 1.0).schedule();

        // Set Climb Positions
        operatorController.povLeft() .onTrue(new SetClimbAngle(climb, ClimbPositions.GRAB));
        operatorController.povRight().onTrue(new SetClimbAngle(climb, ClimbPositions.HANG));
        operatorController.povDown() .onTrue(new SetClimbAngle(climb, ClimbPositions.STOW));

        // Climb manual controls
        // new SetClimbSpeed(climb, operatorController::getLeftY, () -> 0.5, () -> 1.0).schedule();
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }
}