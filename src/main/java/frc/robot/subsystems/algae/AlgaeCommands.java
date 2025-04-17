package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class AlgaeCommands {
    public static Command setPercent(Algae algae, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            algae.setPercent(speed * AdjustableValues.getNumber("Algae_Percent"));
        }, () -> {
            algae.setPercent(0);
        }, algae);
    }

    public static Command setVoltage(Algae algae, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            algae.setVoltage(Volts.of(speed * AdjustableValues.getNumber("Algae_Percent") * RobotController.getInputVoltage()));
        }, () -> {
            algae.setVoltage(Volts.zero());
        }, algae);
    }
}