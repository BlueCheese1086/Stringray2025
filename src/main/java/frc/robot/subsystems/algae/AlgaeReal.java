
package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TurboLogger;

public class AlgaeReal extends Algae {
    private TalonFX algae;
    private LaserCan laser;

    public AlgaeReal(int algaeId, int laserId) {
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
    public void periodic() {
        TurboLogger.log("/Algae/Current", getCurrent().in(Amps));
        TurboLogger.log("/Algae/Percent/Actual", getPercent());
        TurboLogger.log("/Algae/Temperature", getTemperature().in(Celsius));
        TurboLogger.log("/Algae/Voltage/Actual", getVoltage().in(Volts));
        TurboLogger.log("/Algae/LaserStatus", getLaserStatus());
        TurboLogger.log("/Algae/LaserDist", getLaserDistance().in(Meters));
    }

    @Override
    public void setPercent(double percent) {
        TurboLogger.log("/Algae/Percent/Setpoint", percent);

        algae.set(percent);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        TurboLogger.log("/Algae/Voltage/Setpoint", voltage.in(Volts));

        algae.setVoltage(voltage.in(Volts));
    }

    @Override
    public Current getCurrent() {
        return algae.getStatorCurrent().getValue();
    }

    @Override
    public Distance getLaserDistance() {
        Measurement measure = laser.getMeasurement();

        // Returns a ridiculously large measurement if the measure is null or if the reading isn't valid.
        if (measure == null || measure.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return Meters.of(Double.MAX_VALUE);

        return Millimeters.of(measure.distance_mm);
    }

    @Override
    public int getLaserStatus() {
        Measurement measure = laser.getMeasurement();

        // Defaulting to a weak signal status
        if (measure == null) return LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;

        return measure.status;
    }

    @Override
    public double getPercent() {
        return algae.get();
    }

    @Override
    public Temperature getTemperature() {
        return algae.getDeviceTemp().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return algae.getMotorVoltage().getValue();
    }
}
