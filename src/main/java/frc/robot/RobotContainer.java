// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private Elevator elevator;

    public RobotContainer() {
        Drivetrain drivetrain;
        if (Robot.isReal()) {
            new Gyro(new GyroIOPigeon2());
            drivetrain = new Drivetrain(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            new CarriageSubsystem(new CarriageIOSparkMax());
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
        } else {
            drivetrain = new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
            new CarriageSubsystem(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
        }

        AutoFactory autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose, drivetrain::followTrajectory, false, drivetrain);

        autoFactory.trajectoryCmd("My Trajectory");
        autoFactory.newRoutine("My Auto");

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
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(driverController::getLeftX, driverController::getLeftY, driverController::getRightX);
    }
}