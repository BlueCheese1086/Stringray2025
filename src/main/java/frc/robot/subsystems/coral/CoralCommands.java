package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class CoralCommands {
    public static Command setSpeed(Coral coral, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            coral.setPercent(speed * AdjustableValues.getNumber("Coral_Percent"));
        }, () -> {
            coral.setPercent(0);
        }, coral);
    }

    public static Command intakeWithSensor(Coral coral, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            coral.setPercent(throttle.get() * AdjustableValues.getNumber("Coral_Percent"));
        }, () -> {
            coral.setPercent(0);
        }, coral)
            .until(() -> coral.getSensorProximity() < CoralConstants.proximityThreshold);
    }
}