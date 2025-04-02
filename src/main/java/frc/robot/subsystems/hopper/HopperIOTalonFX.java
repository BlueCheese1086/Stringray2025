package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

public class HopperIOTalonFX implements HopperIO {
    private TalonFX track;
    private LaserCan laser;

    public HopperIOTalonFX(int trackId, int laserId) {
        track = new TalonFX(trackId);
        laser = new LaserCan(laserId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        track.getConfigurator().apply(config);

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
        inputs.percent = track.get();
        inputs.voltage = track.getMotorVoltage().getValue();
        inputs.current = track.getStatorCurrent().getValue();
        inputs.temperature = track.getDeviceTemp().getValue();

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
        track.setControl(new DutyCycleOut(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        track.setControl(new VoltageOut(voltage));
    }
}