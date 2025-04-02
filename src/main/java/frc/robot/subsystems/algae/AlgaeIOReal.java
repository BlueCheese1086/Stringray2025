package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

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
        inputs.percent = algae.get();
        inputs.voltage = algae.getMotorVoltage().getValue();
        inputs.current = algae.getStatorCurrent().getValue();
        inputs.temperature = algae.getDeviceTemp().getValue();

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
        algae.set(percent);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        algae.setVoltage(voltage.in(Volts));
    }
}