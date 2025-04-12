package frc.robot.subsystems.algae;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeIOReal implements AlgaeIO {
    private TalonFX algae;
    private LaserCan laser;

    public AlgaeIOReal(int algaeId, int laserId) {
        this.algae = new TalonFX(algaeId);
        this.laser = new LaserCan(laserId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algae.getConfigurator().apply(config);

        // Configuring LaserCan
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            // Default but we have to configure in their app
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
            System.out.println("Algae Laser Can Config Failed!");
        }
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.current = algae.getStatorCurrent().getValueAsDouble();
        inputs.percent = algae.get();
        inputs.temperature = algae.getDeviceTemp().getValueAsDouble();
        inputs.voltage = algae.getMotorVoltage().getValueAsDouble();

        // This can be null, check before using
        Measurement measure = laser.getMeasurement();

        if (measure == null) return;

        inputs.laserStatus = measure.status;

        // Only updating the reading if the sensor has a good read.
        if (inputs.laserStatus != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return;

        inputs.laserReading = measure.distance_mm / 1000.0;
    }

    @Override
    public void setPercent(double percent) {
        algae.set(percent);
    }

    @Override
    public void setVoltage(double voltage) {
        algae.setVoltage(voltage);
    }
}