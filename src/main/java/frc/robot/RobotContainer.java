package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.*;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.commands.SetPosition;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.vision.*;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private Carriage carriage;
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Gyro gyro;
    private Vision vision;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            gyro = new Gyro(new GyroIOPigeon2(RobotMap.GYRO_Pigeon2Id));
            vision = new Vision(new CameraIOReal("", new Transform3d()), new CameraIOReal("", new Transform3d()));
            drivetrain = new Drivetrain(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            carriage = new Carriage(new CarriageIOSparkMax(RobotMap.CARRIAGE_MOTOR_ID, RobotMap.CARRIAGE_CANANDCOLOR_ID));
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
        } else {
            vision = new Vision(new CameraIOSim("", new Transform3d()), new CameraIOSim("", new Transform3d()));
            drivetrain = new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
            carriage = new Carriage(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
        }

        // Assigning default commands
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX));

        // Prepping Choreo
        // AutoFactory autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
        // autoFactory.trajectoryCmd("My Trajectory");
        // autoFactory.newRoutine("My Auto").cmd();

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {
        operatorController.a().whileTrue(new RunCarriage(carriage, 1));
        operatorController.b().whileTrue(new RunCarriage(carriage, -1));

        // Move up/down with pov up/down
        //driverController.povUp().onTrue(new MoveUp(elevator));
        //driverController.povDown().onTrue(new MoveDown(elevator));

        // Button per state
        operatorController.povUp().onTrue(new SetPosition(ElevatorPosition.L4, elevator));
        operatorController.povLeft().onTrue(new SetPosition(ElevatorPosition.L3, elevator));
        operatorController.povRight().onTrue(new SetPosition(ElevatorPosition.L2, elevator));
        operatorController.povDown().onTrue(new SetPosition(ElevatorPosition.STOW, elevator));

        // driverController.a().whileTrue(new PathfindToPose());
        driverController.x().toggleOnTrue(new XStates(drivetrain));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getTeleopCommand() {
        return null;
    }
}
