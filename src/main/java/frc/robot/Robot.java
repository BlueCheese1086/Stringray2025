// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command teleopCommand;

    CommandTracker tracker = new CommandTracker("Commands");

    public Robot() {
        RobotContainer robotContainer = new RobotContainer();

        autonomousCommand = robotContainer.getAutonomousCommand();
        teleopCommand = robotContainer.getTeleopCommand();

        if (autonomousCommand == null) {
            autonomousCommand = Commands.print("No autonomous command configured.");
        }

        if (teleopCommand == null) {
            teleopCommand = Commands.print("No teleop command configured.");
        }

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    /** Runs every tick while the robot is on. */
    @Override
    public void robotPeriodic() {
        // Running the scheduled commands
        CommandScheduler.getInstance().run();
    }

    /** Runs once when the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    /** Runs every tick while the robot is in Disabled mode. */
    @Override
    public void disabledPeriodic() {}

    /** Runs once when the robot exits Disabled mode. */
    @Override
    public void disabledExit() {}

    /** Runs once when the robot enters Autonomous mode. */
    @Override
    public void autonomousInit() {
        autonomousCommand.schedule();
    }

    /** Runs every tick while the robot is in Autonomous mode. */
    @Override
    public void autonomousPeriodic() {}

    /** Runs once when the robot exits Autonomous mode. */
    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    /** Runs once when the robot enters Teleop mode. */
    @Override
    public void teleopInit() {
        teleopCommand.schedule();
    }

    /** Runs every tick while the robot is in Teleop mode. */
    @Override
    public void teleopPeriodic() {
        Logger.recordOutput("/Commands/Unscheduled", tracker.getUnscheduledCommands());
        Logger.recordOutput("/Commands/Scheduled", tracker.getScheduledCommands());

        tracker.getAllCommands().forEach((key, val) -> {
            Logger.recordOutput(String.format("/Commands/All/%s", key), val);
        });


    }

    /** Runs once when the robot exits Teleop mode. */
    @Override
    public void teleopExit() {
        teleopCommand.cancel();
    }

    /** Runs once when the robot enters Test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** Runs every tick while the robot is in Test mode. */
    @Override
    public void testPeriodic() {}

    /** Runs once when the robot leaves Test mode. */
    @Override
    public void testExit() {}
}