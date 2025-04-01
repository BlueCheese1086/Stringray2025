package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Millimeters;

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
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

/** Add your docs here. */
public class CoralIOReal implements CoralIO {
    private TalonFX coral;
    private Canandcolor sensor;
    private LaserCan laser;

    /**
     * Creates a new carriage subsystem with real hardware.
     * 
     * @param coralId  The CAN ID of the {@link TalonFX} motor that puts coral on the reef.
     * @param sensorId The CAN ID of the {@link CANandcolor} sensor to read.
     * @param laserId  The CAN ID of the {@link LaserCan} sensor under the roller.
     */
    public CoralIOReal(int coralId, int sensorId, int laserId) {
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
    public void updateInputs(CoralIOInputs inputs) {
        inputs.percent = coral.get();
        inputs.voltage = coral.getMotorVoltage().getValue();
        inputs.current = coral.getStatorCurrent().getValue();
        inputs.temperature = coral.getDeviceTemp().getValue();

        inputs.sensorProximity = sensor.getProximity();
        inputs.sensorColor = String.format("#%x%x%x", (int) (sensor.getRed() * 255), (int) (sensor.getGreen() * 255), (int) (sensor.getBlue() * 255));

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
        coral.setControl(new DutyCycleOut(percent));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        coral.setControl(new VoltageOut(voltage));
    }
}