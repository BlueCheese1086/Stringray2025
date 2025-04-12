package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AntiTip extends Command {
    private Consumer<Double> setHeight;
    private Supplier<Double> pitchSupplier;
    private Supplier<Double> rollSupplier;

    /**
     * Creates a new {@link AntiTip} command.
     * It checks the roll and pitch of the robot and moves the elevator down to 0 if it is above some threshold.
     * The units of the setHeight parameter don't matter as 0 is 0 in every distance-related unit.
     * 
     * @param setHeight A function that sets the height of the robot's elevator.
     * @param pitchSupplier A function that gets the pitch of the robot in degrees.
     * @param rollSupplier A function that gets the roll of the robot in degrees.
     */
    public AntiTip(Consumer<Double> setHeight, Supplier<Double> pitchSupplier, Supplier<Double> rollSupplier) {
        this.setHeight = setHeight;
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        if (pitchSupplier.get() >= Constants.TipThreshold || rollSupplier.get() >= Constants.TipThreshold) {
            setHeight.accept(0.0);
        }
    }
}