package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeIOReal implements AlgaeIO {
    private TalonFX algae;

    public AlgaeIOReal(int algaeID) {
        this.algae = new TalonFX(algaeID);

        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        algaeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        algaeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algae.getConfigurator().apply(algaeConfig);
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.percent = algae.get();
        inputs.voltage = algae.getMotorVoltage().getValue();
        inputs.current = algae.getStatorCurrent().getValue();
        inputs.temperature = algae.getDeviceTemp().getValue();
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