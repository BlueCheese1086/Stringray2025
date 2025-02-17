// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driverController.button(1).whileTrue(Commands.print("command 1"));
        driverController.button(2).whileTrue(Commands.print("command 2"));
        driverController.button(3).whileTrue(Commands.print("command 3"));
        driverController.button(4).toggleOnTrue(Commands.print("command 4"));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command getTeleopCommand() {
        return Commands.print("No teleop command configured");
    }
}