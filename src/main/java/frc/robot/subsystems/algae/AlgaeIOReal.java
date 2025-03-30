// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

/** Add your docs here. */
public class AlgaeIOReal implements AlgaeIO{

    private TalonFX algae;
    // Creating and applying the config for the algae motor
    TalonFXConfiguration algaeConfig = new TalonFXConfiguration();

    public AlgaeIOReal(int algaeID) {
        this.algae = new TalonFX(algaeID);

        algaeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        algaeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algae.getConfigurator().apply(algaeConfig);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs){
        inputs.algaePercent = algae.get();
        inputs.algaeVoltage = algae.getMotorVoltage().getValue();
        inputs.algaeCurrent = algae.getStatorCurrent().getValue();
        inputs.algaeTemperature = algae.getDeviceTemp().getValue();
    }

    @Override
    public void setVolts(double percent) {
        algae.setVoltage(percent);
    }

}
