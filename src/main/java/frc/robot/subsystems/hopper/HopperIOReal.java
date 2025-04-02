package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

public class HopperIOReal implements HopperIO {
    private SparkMax track;
    private LaserCan laser;

    public HopperIOReal(int trackId, int laserId) {
        track = new SparkMax(trackId, MotorType.kBrushless);
        laser = new LaserCan(laserId);

        SparkMaxConfig trackConfig = new SparkMaxConfig();
        trackConfig.inverted(false);
        trackConfig.idleMode(IdleMode.kBrake);

        track.configure(trackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuring LaserCan
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            // Default but we have to configure in their app
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
            System.out.println("Hopper Laser Can Config Failed!");
        }
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.percent = track.getAppliedOutput();
        inputs.voltage = Volts.of(track.getAppliedOutput() * track.getBusVoltage());
        inputs.current = Amps.of(track.getOutputCurrent());
        inputs.temperature = Celsius.of(track.getMotorTemperature());

        // This can be null, check before using
        Measurement measure = laser.getMeasurement();

        if (Objects.isNull(measure)) return;

        inputs.laserStatus = measure.status;

        // Only updating the reading if the sensor has a good read.
        if (inputs.laserStatus != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return;

        inputs.laserReading = Millimeters.of(measure.distance_mm);
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