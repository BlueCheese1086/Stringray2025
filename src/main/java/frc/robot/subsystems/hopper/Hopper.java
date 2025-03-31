// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Hopper extends SubsystemBase {
    public HopperIO io;
    public HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO io){
        this.io = io;
    }

    public void setTrackPercent(double percent){
        io.setTrackPercent(percent);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("/RealOutputs/Subsystems/Hopper", inputs);
    }
}
