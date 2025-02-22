// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.carriage.*;
import frc.robot.subsystems.carriage.commands.*;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gyro.*;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private Elevator elevator;

    public RobotContainer() {
        if (Robot.isReal()) {
            new Gyro(new GyroIOPigeon2());
            new Drivetrain(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
            new CarriageSubsystem(new CarriageIOSparkMax());
            elevator = new Elevator(new ElevatorIOReal(RobotMap.ELEV_LeftId, RobotMap.ELEV_RightId));
        } else {
            new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
            new CarriageSubsystem(new CarriageIOSim());
            elevator = new Elevator(new ElevatorIOSim());
        }

        configureBindings();
    }

    private void configureBindings() {
        // driverController.button(1).whileTrue(Commands.print("command 1"));
        // driverController.button(2).whileTrue(Commands.print("command 2"));
        // driverController.button(3).whileTrue(Commands.print("command 3"));
        // driverController.button(4).toggleOnTrue(Commands.print("command 4"));
        driverController.a().whileTrue(new RunCarriage(1));
        driverController.b().whileTrue(new RunCarriage(-1));
        
        driverController.povUp().onTrue(new InstantCommand(() -> elevator.setPosition(1), elevator));
        driverController.povLeft().onTrue(new InstantCommand(() -> elevator.setPosition(0.6), elevator));
        driverController.povRight().onTrue(new InstantCommand(() -> elevator.setPosition(0.4), elevator));
        driverController.povDown().onTrue(new InstantCommand(() -> elevator.setPosition(0.1), elevator));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(driverController::getLeftX, driverController::getLeftY, driverController::getRightX);
    }
}