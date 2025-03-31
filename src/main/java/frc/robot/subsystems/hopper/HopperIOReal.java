// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class HopperIOReal implements HopperIO {
    SparkMax track;

    // Creating and applying the config for the track motor
    SparkMaxConfig trackConfig = new SparkMaxConfig();

    public HopperIOReal(int trackID) {
        track = new SparkMax(trackID, MotorType.kBrushless);

        trackConfig.inverted(false);
        trackConfig.idleMode(IdleMode.kBrake);

        track.configure(trackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.trackPercent = track.getAppliedOutput();
        inputs.trackVoltage = Volts.of(track.getAppliedOutput() * track.getBusVoltage());
        inputs.trackCurrent = Amps.of(track.getOutputCurrent());
        inputs.trackTemperature = Celsius.of(track.getMotorTemperature());
    }

    @Override
    public void setTrackPercent(double percent) {
        track.set(percent);
    }
}
