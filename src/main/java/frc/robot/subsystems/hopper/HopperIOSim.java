// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class HopperIOSim implements HopperIO {
    private DCMotorSim hopper;

    public HopperIOSim() {
        this.hopper = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.02, 1), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        hopper.update(0.02);
        inputs.trackPercent = hopper.getInputVoltage() / RobotController.getInputVoltage();
        inputs.trackVoltage = Volts.of(hopper.getInputVoltage());
        inputs.trackCurrent = Amps.of(hopper.getCurrentDrawAmps());
    }

    @Override
    public void setTrackPercent(double percent) {
        hopper.setInputVoltage(percent * RobotController.getInputVoltage());
    }
}
