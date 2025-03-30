package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.OverideCarriage;
import frc.robot.subsystems.carriage.commands.RunAlgaeRoller;
import frc.robot.subsystems.carriage.commands.RunCoralRoller;
import frc.robot.subsystems.carriage.commands.RunIntakeTrack;
import frc.robot.subsystems.carriage.commands.RunSensorOrientedCarriage;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.commands.SetClimbAngle;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drive.Commands.AutoLeftFind;
import frc.robot.subsystems.drive.Commands.DriveCommands;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.util.AntiTip;
import frc.robot.subsystems.util.RecordPose;
import frc.robot.subsystems.vision.*;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private Carriage carriage;
    private Drive drive;
    private Elevator elevator;
    private Gyro gyro;
    private Vision vision;
    private Climb climb;

    // private PathFindToLeft pathFindingLeft;
    // private PathFindToRight pathFindingRight;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            // gyro = new Gyro(new GyroIOPigeon2(Constants.RobotMap.GYRO_Pigeon2Id));
            vision = new Vision(
                    new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
            drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft), new ModuleIOTalonFX(TunerConstants.BackRight));
            carriage = new Carriage(
                    new CarriageIOReal(Constants.RobotMap.CARRIAGE_AlgaeId, Constants.RobotMap.CARRIAGE_CoralId,
                            Constants.RobotMap.CARRIAGE_TrackId, Constants.RobotMap.CARRIAGE_CoralLaserId,
                            Constants.RobotMap.CARRIAGE_AlgaeLaserId));
            elevator = new Elevator(
                    new ElevatorIOReal(Constants.RobotMap.ELEV_LeftId, Constants.RobotMap.ELEV_RightId));
            climb = new Climb(new ClimbIOReal(Constants.RobotMap.CLIMB_MotorId));

        } else {
            drive = new Drive(new GyroIO() {
            },
                    new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontLeft));
            vision = new Vision(
                    new CameraIOReal(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                    new CameraIOReal(VisionConstants.rCameraName,
                            VisionConstants.rCameraTransform));
            carriage = new Carriage(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
            climb = new Climb(new ClimbIOSim());
        }

        // Anti-Tip command (Cancels if the A button is pressed)
        if (RobotBase.isReal()) {
            // new RunIntakeTrack(carriage, ()-> 1.0);
            // new RunCoralRoller(carriage, ()-> 1.0);
        }

        // Assigning default commands

        // Creating the pathfinding command
        // It has an override condition that causes it to stop when the left joystick
        // gets any input.
        // It is defined this way so that you can change the pose it pathfinds to.
        // pathFindingLeft = new PathFindToLeft(drivetrain, () -> {
        // return Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getLeftX).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getLeftY).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getRightX).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getRightY).get(), 0.1)) > 0;
        // });

        // pathFindingRight = new PathFindToRight(drivetrain, () -> {
        // return Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getLeftX).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getLeftY).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getRightX).get(), 0.1)) > 0 ||
        // Math.abs(MathUtil.applyDeadband(((Supplier<Double>)
        // driverController::getRightY).get(), 0.1)) > 0;
        // });

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
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> 0.1,
                        () -> 1));

        driverController.leftBumper().whileTrue(
                DriveCommands.joystickDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> 0.1,
                        () -> 0.2));

        driverController.y().onTrue(new RecordPose(drive));

        if (RobotBase.isReal()) {
            driverController.b().onTrue(Commands.runOnce(() -> gyro.reset(), gyro));
        }

        driverController.start().onTrue(new AutoLeftFind(drive, true)); // False is red

        // Presision Mode = Left
        // driverController.leftBumper().whileTrue(Commands.runOnce(() ->
        // driveCommand.setScalar(Constants.PrecisionScalar)));
        // driverController.leftBumper().onFalse(Commands.runOnce(() ->
        // driveCommand.setScalar(1)));

        // //Presion
        // driverController.rightBumper().whileTrue(Commands.runOnce(() ->
        // driveCommand.setScalar(Constants.PrecisionScalar)));
        // driverController.rightBumper().onFalse(Commands.runOnce(() ->
        // driveCommand.setScalar(1)));

        // //PathFind: Overide is joystick
        // driverController.back().onTrue(pathFindingLeft);
        // driverController.start().onTrue(pathFindingRight);

        // Intake Coral & Algae
        // driverController.leftTrigger(0.2).whileTrue(new
        // RunSensorOrientedCarriage(carriage, () -> -.1));
        // driverController.leftTrigger(0.2).whileTrue(new RunAlgaeRoller(carriage, ()
        // -> -.1));

        // Outtake Coral & Algae (works)
        // driverController.rightTrigger(0.2).whileTrue(new
        // RunSensorOrientedCarriage(carriage, () -> -driverController.getRightY()));
        // driverController.rightTrigger(0.2).whileTrue(new RunAlgaeRoller(carriage, ()
        // -> -driverController.getRightY()));
        // driverController.leftTrigger(0.2).whileTrue(new RunIntakeTrack(carriage, ()
        // -> -driverController.getLeftY()));

        // // Overide Shoot
        // driverController.y().toggleOnTrue(new OverideCarriage(carriage, () -> 1.0));

        // // Operator Buttons

        // // Reset Encoder
        // operatorController.start().onTrue(elevator.resetEncoder());
        // operatorController.back().onTrue(elevator.resetEncoder());

        // Stow for Elevator
        operatorController.leftBumper()
                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));
        operatorController.leftTrigger(0.2)
                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));

        // Set Elevator Height
        operatorController.a().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L1), elevator));
        operatorController.b().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2), elevator));
        operatorController.x().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3), elevator));
        operatorController.y().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L4), elevator));

        // L3 and L2 Elevator height
        operatorController.rightBumper()
                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3Algae), elevator));
        operatorController.rightTrigger(0.2)
                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2Algae), elevator));
        // Climb 5.7 degrees
        // Elevator manual controls
        operatorController.axisMagnitudeGreaterThan(5, 0.1)
                .whileTrue(Commands.run(() -> elevator.setVolts(Volts
                        .of((operatorController.getRightY() - Math.copySign(0.1, operatorController.getRightY())) * 6)),
                        elevator));

        operatorController.axisMagnitudeGreaterThan(1, 0.1)
                .whileTrue(Commands.run(() -> climb.setVolts(Volts
                        .of((operatorController.getLeftY() - Math.copySign(0.1, operatorController.getLeftY())) * 6)),
                        elevator));

        // Climb Controls
        // operatorController.povLeft().whileTrue(new SetClimbAngle(climb,
        // ClimbConstants.extended));
        // operatorController.povRight().whileTrue(new SetClimbAngle(climb,
        // ClimbConstants.tucked));
        // operatorController.povDown().whileTrue(new SetClimbAngle(climb,
        // ClimbConstants.stow));
    }

    public void periodic() {
        // Logger.recordOutput("/PlaceToGo", drivetrain.getClosestReefPoint());
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("middle");
    }
}
