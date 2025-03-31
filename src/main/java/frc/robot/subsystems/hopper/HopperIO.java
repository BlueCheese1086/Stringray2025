// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface HopperIO{
    @AutoLog
    public static class HopperIOInputs{
        public double trackPercent = 0;
        public Voltage trackVoltage = Volts.zero();
        public Current trackCurrent = Amps.zero();
        public Temperature trackTemperature = Celsius.zero();
    }

    public void updateInputs(HopperIOInputs inputs);

    public void setTrackPercent(double percent);
}
