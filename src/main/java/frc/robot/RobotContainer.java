// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
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

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    public Carriage carriage;
    public Drivetrain drivetrain;
    public Elevator elevator;
    public Gyro gyro;

    public RobotContainer() {
        // Initializing subsystems
        if (Robot.isReal()) {
            gyro = new Gyro(new GyroIOPigeon2(RobotMap.GYRO_Pigeon2Id));
            drivetrain = new Drivetrain(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            carriage = new Carriage(new CarriageIOSparkMax());
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
        } else {
            drivetrain = new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
            carriage = new Carriage(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
        }

        // Assigning default commands
        drivetrain.setDefaultCommand(new SwerveDrive(driverController::getLeftX, driverController::getLeftY, driverController::getRightX));

        // Prepping Choreo
        // AutoFactory autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
        // autoFactory.trajectoryCmd("My Trajectory");
        // autoFactory.newRoutine("My Auto").cmd();

        // Configuring controller bindings
        configureBindings();
    }

    private void configureBindings() {
        operatorController.a().whileTrue(new RunCarriage(1));
        operatorController.b().whileTrue(new RunCarriage(-1));

        // Move up/down with pov up/down
        //driverController.povUp().onTrue(new MoveUp(elevator));
        //driverController.povDown().onTrue(new MoveDown(elevator));

        // Button per state
        operatorController.povUp().onTrue(new SetPosition(ElevatorPosition.L4, elevator));
        operatorController.povLeft().onTrue(new SetPosition(ElevatorPosition.L3, elevator));
        operatorController.povRight().onTrue(new SetPosition(ElevatorPosition.L2, elevator));
        operatorController.povDown().onTrue(new SetPosition(ElevatorPosition.STOW, elevator));

        // driverController.a().whileTrue(new PathfindToPose());
        driverController.x().toggleOnTrue(new XStates());
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getTeleopCommand() {
        return null;
    }
}