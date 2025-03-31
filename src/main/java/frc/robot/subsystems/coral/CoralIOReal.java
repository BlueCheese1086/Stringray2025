package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;

import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class CoralIOReal implements CoralIO {
    private TalonFX coral;
    private Canandcolor sensor;
    // private LaserCan laser;

    /**
     * Creates a new carriage subsystem with real hardware.
     * 
     * @param algaeId The CAN id of the {@link TalonFX} motor that moves the algae.
     * @param coralId The CAN id of the {@link TalonFX} motor that puts coral on the reef.
     * @param trackId The CAN id of the {@link SparkMax} motor that runs in the chute.
     * @param sensorId The CAN id of the {@link CANandcolor} sensor to read.
     */
    public CoralIOReal(int coralId, int sensorId, int trackLaserid) {
        coral = new TalonFX(coralId);
        sensor = new Canandcolor(sensorId);
        // laser = new LaserCan(trackLaserid);

        // Creating and applying the config for the coral motor
        TalonFXConfiguration coralConfig = new TalonFXConfiguration();
        coralConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        coralConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coral.getConfigurator().apply(coralConfig);
        


        // Create laser can Configs
        // try {
        //     laser.setRangingMode(LaserCan.RangingMode.SHORT);
        //     laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16)); //Defualt But we have to Configure in Their App
        //     laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        // } catch (ConfigurationFailedException e) {
        //     e.printStackTrace();
        //     System.out.println("Laser Can Config Failed!");
        // }
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        inputs.percent = coral.get();
        inputs.voltage = coral.getMotorVoltage().getValue();
        inputs.current = coral.getStatorCurrent().getValue();
        inputs.temperature = coral.getDeviceTemp().getValue();

        inputs.sensorProximity = sensor.getProximity();
        inputs.sensorColor = String.format("#%x%x%x", (int) (sensor.getRed() * 255), (int) (sensor.getGreen() * 255), (int) (sensor.getBlue() * 255));

        // inputs.algaeLaserMeasurement = Millimeters.of(laser.getMeasurement().distance_mm);
        // inputs.alageWeakSignal = LaserCan.LASERCAN_STATUS_WEAK_SIGNAL;
        // inputs.algaeValidMeasurement = LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;

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