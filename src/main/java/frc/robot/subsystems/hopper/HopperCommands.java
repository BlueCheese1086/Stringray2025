package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class HopperCommands {
    public static Command setPercent(Hopper hopper, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            hopper.setPercent(speed * AdjustableValues.getNumber("Hopper_Percent"));
        }, () -> {
            hopper.setPercent(0);
        }, hopper);
    }

    public static Command setVoltage(Hopper hopper, Supplier<Double> throttle) {
        return Commands.runEnd(() -> {
            double speed = throttle.get();

            speed = MathUtils.applyDeadbandWithOffsets(speed, Constants.deadband);
            speed = Math.copySign(speed * speed, speed);

            hopper.setVoltage(Volts.of(speed * AdjustableValues.getNumber("Hopper_Percent") * RobotController.getInputVoltage()));
        }, () -> {
            hopper.setVoltage(Volts.zero());
        }, hopper);
    }
}