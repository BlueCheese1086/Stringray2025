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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

public class HopperReal extends Hopper {
    private TalonFX track;
    private LaserCan laser;

    // Control methods
    private DutyCycleOut percentControl = new DutyCycleOut(0);
    private VoltageOut voltageControl = new VoltageOut(0);

    private int laserStatus = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
    private Distance laserReading = Millimeters.zero();

    public HopperReal(int trackId, int laserId) {
        track = new TalonFX(trackId);
        laser = new LaserCan(laserId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        track.getConfigurator().apply(config);

        // Configuring LaserCan
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            // Default but we have to configure in their app ???
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportWarning("Hopper Laser Can Config Failed!", false);
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // This can be null, check before using
        Measurement measure = laser.getMeasurement();

        if (measure == null) return;

        laserStatus = measure.status;

        // Only updating the reading if the sensor has a good read.
        if (laserStatus != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return;

        laserReading = Millimeters.of(measure.distance_mm);
    }

    @Override
    public Current getCurrent() {
        return track.getStatorCurrent().getValue();
    }

    @Override
    public double getPercent() {
        return track.getDutyCycle().getValue();
    }

    @Override
    public Temperature getTemperature() {
        return track.getDeviceTemp().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return track.getMotorVoltage().getValue();
    }

    @Override
    public Distance getLaserReading() {
        return laserReading;
    }

    @Override
    public int getLaserStatus() {
        return laserStatus;
    }

    @Override
    public void setPercent(double percent) {
        track.setControl(percentControl.withOutput(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        track.setControl(voltageControl.withOutput(voltage));
    }
}