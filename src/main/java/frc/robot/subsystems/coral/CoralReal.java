package frc.robot.subsystems.coral;

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
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Objects;

public class CoralReal extends Coral {
    private TalonFX coral;
    private Canandcolor sensor;
    private LaserCan laser;

    /**
     * Creates a new coral subsystem with real hardware.
     * 
     * @param coralId  The CAN ID of the {@link TalonFX} motor that puts coral on the reef.
     * @param sensorId The CAN ID of the {@link CANandcolor} sensor to read.
     * @param laserId  The CAN ID of the {@link LaserCan} sensor under the roller.
     */
    public CoralReal(int coralId, int sensorId, int laserId) {
        coral = new TalonFX(coralId);
        sensor = new Canandcolor(sensorId);
        laser = new LaserCan(laserId);

        // Creating and applying the config for the coral motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coral.getConfigurator().apply(config);

        // Configuring LaserCan
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            // Default but we have to configure in their app
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
            System.out.println("Coral Laser Can Config Failed!");
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Coral/Percent/Actual", getPercent());
        SmartDashboard.putNumber("/Coral/Voltage/Actual", getVoltage().in(Volts));
        SmartDashboard.putNumber("/Coral/Current", getCurrent().in(Amps));
        SmartDashboard.putNumber("/Coral/Temperature", getTemperature().in(Celsius));
        SmartDashboard.putNumber("/Coral/SensorProximity", getSensorProximity());
        SmartDashboard.putString("/Coral/SensorColor", getSensorColor());
        SmartDashboard.putNumber("/Coral/LaserStatus", getLaserStatus());
        SmartDashboard.putNumber("/Coral/LaserDistance", getLaserReading().in(Inches));
    }

    @Override
    public void setPercent(double percent) {
        SmartDashboard.putNumber("/Coral/Percent/Setpoint", percent);

        coral.setControl(new DutyCycleOut(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        SmartDashboard.putNumber("/Coral/Voltage/Setpoint", voltage.in(Volts));

        coral.setControl(new VoltageOut(voltage));
    }

    /** Gets the proximity read by the CANandColor. */
    public double getSensorProximity() {
        return sensor.getProximity();
    }

    /** Gets the color seen by the CANandColor. */
    public String getSensorColor() {
        return String.format("#%x%x%x", (int) (sensor.getRed() * 255), (int) (sensor.getGreen() * 255), (int) (sensor.getBlue() * 255));
    }

    /** Gets the distance seen by the LaserCan. */
    public Distance getLaserReading() {
        // This can be null, check before using
        Measurement measure = laser.getMeasurement();

        if (Objects.isNull(measure) || measure.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return Meters.of(Double.MAX_VALUE);

        return Millimeters.of(measure.distance_mm);
    }

    /** Gets the latest status of the LaserCan. */
    public int getLaserStatus() {
        // This can be null, check before using
        Measurement measure = laser.getMeasurement();

        if (Objects.isNull(measure)) return LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;

        return measure.status;
    }

    /** Gets the percent output of the coral motor. */
    public double getPercent() {
        return coral.get();
    }

    /** Gets the voltage output of the coral motor. */
    public Voltage getVoltage() {
        return coral.getMotorVoltage().getValue();
    }

    /** Gets the current applied to the coral motor. */
    public Current getCurrent() {
        return coral.getStatorCurrent().getValue();
    }

    /** Gets the internal temperature of the coral motor. */
    public Temperature getTemperature() {
        return coral.getDeviceTemp().getValue();
    }
}