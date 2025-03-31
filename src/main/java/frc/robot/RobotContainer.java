package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIOReal;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.commands.SetAlgaePercent;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.OverideCarriage;
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
import frc.robot.subsystems.drive.Commands.AutoRightFind;
import frc.robot.subsystems.drive.Commands.DriveCommands;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.commands.SetHopperPercent;
import frc.robot.subsystems.util.RecordPose;
import frc.robot.subsystems.vision.*;

import static edu.wpi.first.units.Units.Volts;

public class RobotContainer {
        private CommandXboxController driverController = new CommandXboxController(0);
        private CommandXboxController operatorController = new CommandXboxController(1);

        private Carriage carriage;
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
                        // gyro = new Gyro(new GyroIOPigeon2(Constants.RobotMap.GYRO_Pigeon2Id));
                        vision = new Vision(
                                        new CameraIOReal(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                                        new CameraIOReal(VisionConstants.rCameraName,
                                                        VisionConstants.rCameraTransform));
                        drive = new Drive(
                                        new GyroIOPigeon2(),
                                        vision,
                                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                                        new ModuleIOTalonFX(TunerConstants.BackRight));
                        algae = new Algae(new AlgaeIOReal(Constants.RobotMap.CARRIAGE_AlgaeId));
                        hopper = new Hopper(new HopperIOReal(Constants.RobotMap.HOPPER_trackiD));
                        carriage = new Carriage(
                                        new CarriageIOReal(Constants.RobotMap.CARRIAGE_CoralId,
                                                        Constants.RobotMap.CARRIAGE_CoralLaserId,
                                                        Constants.RobotMap.CARRIAGE_AlgaeLaserId));
                        elevator = new Elevator(
                                        new ElevatorIOReal(Constants.RobotMap.ELEV_LeftId,
                                                        Constants.RobotMap.ELEV_RightId));
                        climb = new Climb(new ClimbIOReal(Constants.RobotMap.CLIMB_MotorId));

                } else {
                        vision = new Vision(
                                        new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
                                        new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
                        drive = new Drive(new GyroIO() {
                        },
                                        vision,
                                        new ModuleIOSim(TunerConstants.FrontLeft),
                                        new ModuleIOSim(TunerConstants.FrontLeft),
                                        new ModuleIOSim(TunerConstants.FrontLeft),
                                        new ModuleIOSim(TunerConstants.FrontLeft));
                        carriage = new Carriage(new CarriageIOSim());
                        hopper = new Hopper(new HopperIOSim());
                        elevator = new Elevator(new ElevatorIOSim());
                        algae = new Algae(new AlgaeIOSim());
                        climb = new Climb(new ClimbIOSim());
                }

                // Anti-Tip command (Cancels if the A button is pressed)
                if (RobotBase.isReal()) {
                }

                // Configuring controller bindings
                configureBindings();
        }

        private void configureBindings() {

                //Defualt Commands
                hopper.setDefaultCommand(new SetHopperPercent(hopper, ()-> 0.1));
                carriage.setDefaultCommand(new RunSensorOrientedCarriage(carriage));

                // Driver Controls
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY(),
                                                () -> driverController.getLeftX(),
                                                () -> -driverController.getRightX(),
                                                () -> 0.1,
                                                () -> 1));

                // Presice Mode
                driverController.leftBumper().whileTrue(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY(),
                                                () -> driverController.getLeftX(),
                                                () -> -driverController.getRightX(),
                                                () -> 0.1,
                                                () -> 0.2));

                driverController.rightBumper().whileTrue(
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

                // Path Find / Overide is joystick
                driverController.back().onTrue(new AutoLeftFind(drive, true)); // False is red
                driverController.start().onTrue(new AutoRightFind(drive, true));

                // Intake Coral & Algae
                driverController.leftTrigger(0.2).whileTrue(new OverideCarriage(carriage, () -> driverController.getLeftTriggerAxis())); // Need to figure out the right voltage in order to intake 
                driverController.leftTrigger(0.2).whileTrue(new SetAlgaePercent(algae, () -> driverController.getLeftTriggerAxis()));

                // Outtake Coral & Algae (works)
                driverController.rightTrigger(0.2).whileTrue(new OverideCarriage(carriage, () -> driverController.getRightTriggerAxis()));
                driverController.rightTrigger(0.2).whileTrue(new SetAlgaePercent(algae, () -> driverController.getRightTriggerAxis()));



                // Operator Buttons

                // Reset Encoder
                operatorController.start().onTrue(elevator.resetEncoder());
                operatorController.back().onTrue(elevator.resetEncoder());

                // Stow for Elevator
                operatorController.leftBumper()
                                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.STOW), elevator));
                operatorController.leftTrigger(0.2)
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
                                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L3Algae),
                                                elevator));
                operatorController.rightTrigger(0.2)
                                .onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorPositions.L2Algae),
                                                elevator));
                // Climb 5.7 degrees
                // Elevator manual controls
                operatorController.axisMagnitudeGreaterThan(5, 0.1)
                                .whileTrue(Commands.run(() -> elevator.setVolts(Volts
                                                .of((operatorController.getRightY()
                                                                - Math.copySign(0.1, operatorController.getRightY()))
                                                                * 6)),
                                                elevator));

                operatorController.axisMagnitudeGreaterThan(1, 0.1)
                                .whileTrue(Commands.run(() -> climb.setVolts(Volts
                                                .of((operatorController.getLeftY()
                                                                - Math.copySign(0.1, operatorController.getLeftY()))
                                                                * 6)),
                                                elevator));

                // Climb Controls
                operatorController.povLeft().whileTrue(new SetClimbAngle(climb,
                                ClimbConstants.extended));
                operatorController.povRight().whileTrue(new SetClimbAngle(climb,
                                ClimbConstants.tucked));
                operatorController.povDown().whileTrue(new SetClimbAngle(climb,
                                ClimbConstants.stow));
        }

        public void periodic() {
                // Logger.recordOutput("/PlaceToGo", drivetrain.getClosestReefPoint());
        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("middle");
        }
}
