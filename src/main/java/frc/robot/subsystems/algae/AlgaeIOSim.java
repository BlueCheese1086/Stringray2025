// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class AlgaeIOSim implements AlgaeIO {
    private DCMotorSim algae;

    public AlgaeIOSim() {
        this.algae = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.02, 1),
                DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.algaePercent = algae.getInputVoltage() / RobotController.getInputVoltage();
        inputs.algaeVoltage = Volts.of(algae.getInputVoltage());
        inputs.algaeCurrent = Amps.of(algae.getCurrentDrawAmps());
    }

    @Override
    public void setVolts(double percent) {
        algae.setInputVoltage(percent * RobotController.getInputVoltage());
    }

}
