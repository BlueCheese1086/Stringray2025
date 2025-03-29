package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.RunAlgaeRoller;
import frc.robot.subsystems.carriage.commands.RunCoralRoller;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.commands.SetClimbAngle;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.vision.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private Carriage carriage;
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Gyro gyro;
    private Vision vision;
    private Climb climb;

    private SwerveDrive driveCommand;
    private PathFindToLeft pathFindingLeft;
    private PathFindToRight pathFindingRight;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            gyro = new Gyro(new GyroIOPigeon2(Constants.RobotMap.GYRO_Pigeon2Id));
            vision = new Vision(
                    new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
            drivetrain = new Drivetrain(gyro, vision, new ModuleIOTalonFX(0), new ModuleIOTalonFX(1),
                    new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            carriage = new Carriage(
                    new CarriageIOReal(Constants.RobotMap.CARRIAGE_CoralId, Constants.RobotMap.CARRIAGE_AlgaeId,
                            Constants.RobotMap.CARRIAGE_TrackId, Constants.RobotMap.CARRIAGE_CoralLaserId));
            elevator = new Elevator(
                    new ElevatorIOReal(Constants.RobotMap.ELEV_LeftId, Constants.RobotMap.ELEV_RightId));
            climb = new Climb(new ClimbIOReal(Constants.RobotMap.CLIMB_MotorId));
        } else {
            vision = new Vision(
                    new CameraIOReal(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOReal(VisionConstants.rCameraName,
                            VisionConstants.rCameraTransform));
            drivetrain = new Drivetrain(gyro, vision, new ModuleIOSim(0), new ModuleIOSim(1),
                    new ModuleIOSim(2), new ModuleIOSim(3));
            carriage = new Carriage(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
            climb = new Climb(new ClimbIOSim());
        }

        // Anti-Tip command (Cancels if the A button is pressed)
        if (RobotBase.isReal()) {
            new AntiTip(drivetrain, elevator, gyro, () -> driverController.getHID().getAButton()).schedule();
        }

        // Creating the drive command
        // Creating it here so that I can pass in an adjustable scalar to limit speeds.
        driveCommand = new SwerveDrive(drivetrain, driverController::getLeftX, driverController::getLeftY,
                driverController::getRightX, driverController.getHID()::getLeftStickButtonPressed);

        // Assigning default commands
        drivetrain.setDefaultCommand(driveCommand);

        // Creating the pathfinding command
        // It has an override condition that causes it to stop when the left joystick
        // gets any input.
        // It is defined this way so that you can change the pose it pathfinds to.
        pathFindingLeft = new PathFindToLeft(drivetrain, () -> {
            return Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftX).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftY).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getRightX).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getRightY).get(), 0.1)) > 0;
        });

        pathFindingRight = new PathFindToRight(drivetrain, () -> {
            return Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftX).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftY).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getRightX).get(), 0.1)) > 0 ||
                    Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getRightY).get(), 0.1)) > 0;
        });

        // Prepping Choreo
        // AutoFactory autoFactory = new AutoFactory(drivetrain::getPose,
        // drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
        // autoFactory.trajectoryCmd("My Trajectory");
        // autoFactory.newRoutine("My Auto").cmd();

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {

        // Driver Controls
        driverController.y().onTrue(new RecordPose(drivetrain));
        driverController.x().onTrue(new XStates(drivetrain));
        if (RobotBase.isReal()) {
            driverController.b().onTrue(Commands.runOnce(() -> gyro.reset(), gyro));
        }

        //Presision Mode = Left
        driverController.leftBumper().whileTrue(Commands.runOnce(() -> driveCommand.setScalar(Constants.PrecisionScalar)));
        driverController.leftBumper().onFalse(Commands.runOnce(() -> driveCommand.setScalar(1)));

        //Presion
        driverController.rightBumper().whileTrue(Commands.runOnce(() -> driveCommand.setScalar(Constants.PrecisionScalar)));
        driverController.rightBumper().onFalse(Commands.runOnce(() -> driveCommand.setScalar(1)));

        //PathFind: Overide is joystick
        driverController.back().onTrue(pathFindingLeft);
        driverController.start().onTrue(pathFindingRight);

        //Intake Coral & Algae
        driverController.leftTrigger(0.2).whileTrue(new RunCoralRoller(carriage, () -> -1.0));
        driverController.leftTrigger(0.2).whileTrue(new RunAlgaeRoller(carriage, () -> -1.0));

        //Outtake Coral & Algae
        driverController.rightTrigger(0.2).whileTrue(new RunCoralRoller(carriage, () -> 1.0));
        driverController.rightTrigger(0.2).whileTrue(new RunAlgaeRoller(carriage, () -> 1.0));

        // Operator Button
        // Reset Encoder
        operatorController.start().onTrue(elevator.resetEncoder());
        operatorController.back().onTrue(elevator.resetEncoder());

        // Stow for Elevator
        operatorController.leftBumper().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));
        operatorController.leftTrigger(0.2).onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));

        //Set Elevator Height
        operatorController.a().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L1), elevator));
        operatorController.b().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2), elevator));
        operatorController.x().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3), elevator));
        operatorController.y().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L4), elevator));

        // L3 and L2 Elevator height
        operatorController.rightBumper().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3Algae), elevator));
        operatorController.rightTrigger(0.2).onTrue(Commands.runOnce(()-> elevator.setPosition(ElevatorPositions.L2Algae), elevator));

        // Climb Controls
        operatorController.povLeft().whileTrue(new SetClimbAngle(climb, ClimbConstants.extended));
        operatorController.povRight().whileTrue(new SetClimbAngle(climb, ClimbConstants.tucked));
        operatorController.povDown().whileTrue(new SetClimbAngle(climb, ClimbConstants.stow));
    }

    public void periodic() {
        Logger.recordOutput("/PlaceToGo", drivetrain.getClosestReefPoint());
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }
}
