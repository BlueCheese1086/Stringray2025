package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.algae.commands.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.commands.*;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.coral.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.hopper.commands.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;

import static edu.wpi.first.units.Units.Volts;

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
                    new CameraIOReal(VisionConstants.rCameraName,
                            VisionConstants.rCameraTransform));
            drive = new Drive(
                    gyro,
                    vision,
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            algae = new Algae(new AlgaeIOReal(RobotMap.ALGAE_MotorId, RobotMap.ALGAE_LaserId));
            hopper = new Hopper(new HopperIOReal(RobotMap.HOPPER_MotorId, RobotMap.HOPPER_LaserId));
            coral = new Coral(
                    new CoralIOReal(RobotMap.CORAL_MotorId,
                            RobotMap.CORAL_SensorId,
                            RobotMap.CORAL_LaserId));
            elevator = new Elevator(
                    new ElevatorIOReal(Constants.RobotMap.ELEV_LeftId,
                            Constants.RobotMap.ELEV_RightId));
            climb = new Climb(new ClimbIOReal(Constants.RobotMap.CLIMB_MotorId));

        } else {
            vision = new Vision(
                    new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
            drive = new Drive(
                    gyro,
                    vision,
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontLeft));
            coral = new Coral(new CoralIOSim());
            hopper = new Hopper(new HopperIOSim());
            elevator = new Elevator(new ElevatorIOSim());
            algae = new Algae(new AlgaeIOSim());
            climb = new Climb(new ClimbIOSim());
        }

        // Anti-Tip command (Cancels if the A button is pressed)
        if (RobotBase.isReal()) {
            // new AntiTip(drive, elevator, gyro, () -> driverController.getHID().getAButton())
        }

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {
        // Default Commands
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> 0.1,
                        () -> 1));

        // Presice Mode
        driverController.leftBumper().or(driverController.rightBumper())
            .whileTrue(
                DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> 0.1,
                        () -> 0.2));

        driverController.y().onTrue(new RecordPose(drive));

        // Path Find / Overide is joystick
        driverController.back()
            .onTrue(new AutoLeftFind(drive, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue));

        driverController.start()
            .onTrue(new AutoRightFind(drive, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue));

        // Intake Coral & Algae
        driverController.leftTrigger(0.2)
            .whileTrue(new SetCoralPercent(coral, () -> driverController.getLeftTriggerAxis()))
            .whileTrue(new SetAlgaePercent(algae, () -> driverController.getLeftTriggerAxis()));

        // Need to figure out the right voltage in order to intake

        // Outtake Coral & Algae (works)
        driverController.rightTrigger(0.2)
            .whileTrue(new SetCoralPercent(coral, () -> driverController.getRightTriggerAxis()))
            .whileTrue(new SetAlgaePercent(algae, () -> driverController.getRightTriggerAxis()));

        // Operator Buttons

        // Reset Encoder
        operatorController.start().or(operatorController.back())
            .onTrue(elevator.resetEncoder());

        // Stow for Elevator
        operatorController.leftBumper().or(operatorController.leftTrigger(0.2))
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));

        // Set Elevator Height
        operatorController.a()
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L1), elevator));
        operatorController.b()
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2), elevator));
        operatorController.x()
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3), elevator));
        operatorController.y()
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L4), elevator));

        // L3 and L2 Elevator height
        operatorController.rightBumper()
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3Algae), elevator));
        operatorController.rightTrigger(0.2)
            .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2Algae), elevator));

        // Climb 5.7 degrees
        // Elevator manual controls
        operatorController.axisMagnitudeGreaterThan(5, 0.1)
            .whileTrue(Commands.run(() -> elevator.setVolts(Volts.of(MathUtils.applyDeadbandWithOffsets(operatorController.getRightY(), 0.1) * 6)), elevator));

        operatorController.axisMagnitudeGreaterThan(1, 0.1)
            .whileTrue(Commands.run(() -> climb.setVolts(Volts.of(MathUtils.applyDeadbandWithOffsets(operatorController.getLeftY(), 0.1) * 6)), climb));

        // Climb Controls
        operatorController.povLeft()
            .whileTrue(new SetClimbAngle(climb, ClimbConstants.extended));
        operatorController.povRight()
            .whileTrue(new SetClimbAngle(climb, ClimbConstants.tucked));
        operatorController.povDown()
            .whileTrue(new SetClimbAngle(climb, ClimbConstants.stow));
    }

    public void periodic() {
        // Logger.recordOutput("/PlaceToGo", drivetrain.getClosestReefPoint());
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }
}
