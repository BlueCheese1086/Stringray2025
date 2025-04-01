package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
    private SparkMax track;

    public HopperIOReal(int trackID) {
        track = new SparkMax(trackID, MotorType.kBrushless);

        SparkMaxConfig trackConfig = new SparkMaxConfig();
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
    public void setPercent(double percent) {
        track.set(percent);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        track.setVoltage(voltage);
    }
}