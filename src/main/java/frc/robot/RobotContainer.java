package frc.robot;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.*;
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

    private PathfindToPose pathfindingCommand;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            gyro = new Gyro(new GyroIOPigeon2(Constants.RobotMap.GYRO_Pigeon2Id));
            vision = new Vision(
                new CameraIOSim(Constants.VisionConstants.lCameraName, Constants.VisionConstants.lCameraTransform),
                new CameraIOSim(Constants.VisionConstants.rCameraName, Constants.VisionConstants.rCameraTransform));
            drivetrain = new Drivetrain(gyro, vision, new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            carriage = new Carriage(new CarriageIOReal(Constants.RobotMap.CARRIAGE_CoralId, Constants.RobotMap.CARRIAGE_AlgaeId, Constants.RobotMap.CARRIAGE_TrackId, Constants.RobotMap.CARRIAGE_CandandColorId));
            elevator = new Elevator(new ElevatorIOReal(Constants.RobotMap.ELEV_LeftId, Constants.RobotMap.ELEV_RightId));
        } else {
            vision = new Vision(
                new CameraIOReal(Constants.VisionConstants.lCameraName, Constants.VisionConstants.lCameraTransform),
                new CameraIOReal(Constants.VisionConstants.rCameraName, Constants.VisionConstants.rCameraTransform));
            drivetrain = new Drivetrain(gyro, vision, new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
            carriage = new Carriage(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
        }

        // Assigning default commands
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController.getHID()::getLeftStickButtonPressed));

        // Creating the pathfinding command
        // It has an override condition that causes it to stop when the left joystick gets any input.
        // It is defined this way so that you can change the pose it pathfinds to.
        pathfindingCommand = new PathfindToPose(drivetrain, new Pose2d(), () -> {
            return Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftX).get(), 0.1)) > 0 ||
                   Math.abs(MathUtil.applyDeadband(((Supplier<Double>) driverController::getLeftY).get(), 0.1)) > 0;
        });

        // Prepping Choreo
        // AutoFactory autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
        // autoFactory.trajectoryCmd("My Trajectory");
        // autoFactory.newRoutine("My Auto").cmd();

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {
        // driverController.a().whileTrue(new RunCarriage(carriage, 1));
        // driverController.b().whileTrue(new RunCarriage(carriage, -1));

        // Move up/down with pov up/down
        operatorController.povDown().onTrue(elevator.setGoal(ElevatorPositions.STOW));

        operatorController.a().onTrue(elevator.setGoal(ElevatorPositions.L1));
        operatorController.b().onTrue(elevator.setGoal(ElevatorPositions.L2));
        operatorController.x().onTrue(elevator.setGoal(ElevatorPositions.L3));
        operatorController.y().onTrue(elevator.setGoal(ElevatorPositions.L4));


        // Button per state
        // operatorController.povUp().onTrue(new SetPosition(Elevator.ElevatorPosition.L4, elevator));
        // operatorController.povLeft().onTrue(new SetPosition(Elevator.ElevatorPosition.L3, elevator));
        // operatorController.povRight().onTrue(new SetPosition(Elevator.ElevatorPosition.L2, elevator));
        // operatorController.povDown().onTrue(new SetPosition(Elevator.ElevatorPosition.INTAKE, elevator));

        // driverController.x().toggleOnTrue(new XStates(drivetrain));
        // driverController.y().onTrue(new RecordPose(drivetrain));

        driverController.rightStick().toggleOnTrue(pathfindingCommand);
    }

    public void periodic() {
        Logger.recordOutput("/PlaceToGo", drivetrain.getClosestReefPoint());

        pathfindingCommand.setGoalPose(drivetrain.getClosestReefPoint());
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }

    public Command getTeleopCommand() {
        return null;
    }
}
