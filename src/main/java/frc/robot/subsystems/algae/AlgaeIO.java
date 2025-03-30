// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface AlgaeIO {

    @AutoLog
    public class AlgaeIOInputs {
        public double algaePercent = 0;
        public Voltage algaeVoltage = Volts.zero();
        public Current algaeCurrent = Amps.zero();
        public Temperature algaeTemperature = Celsius.zero();
    }

    public void updateInputs(AlgaeIOInputs inputs);

    public void set(double percent);
}
