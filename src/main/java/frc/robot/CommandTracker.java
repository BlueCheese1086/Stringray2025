package frc.robot;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;

public class CommandTracker {
    private NetworkTable table;
    private SendableBuilderImpl sendableBuilder;
    private GenericSubscriber nameSubscriber;
    private HashMap<String,Boolean> allCommands;
    
    public CommandTracker(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
        sendableBuilder = new SendableBuilderImpl();

        sendableBuilder.setTable(table);

        CommandScheduler.getInstance().initSendable(sendableBuilder);

        table.getTopic("Names").genericSubscribe("String[]");

        nameSubscriber = sendableBuilder.getTopic("Names").genericSubscribe("string[]");

        allCommands = new HashMap<String,Boolean>();
    }

    public void update() {
        String[] commands = getScheduledCommands();

        // Resetting the allCommands hashmap
        allCommands.forEach((key, value) -> {
            allCommands.put(key, false);
        });

        // Marking commands as scheduled in the hashmap
        for (String command : commands) {
            allCommands.put(command, true);
        }
    }

    public HashMap<String,Boolean> getAllCommands() {
        return allCommands;
    }

    public String[] getScheduledCommands() {
        return nameSubscriber.getStringArray(new String[0]);
    }

    public String[] getUnscheduledCommands() {
        String[] unscheduled = new String[allCommands.size() - getScheduledCommands().length];

        int i = 0;

        for (String key : allCommands.keySet()) {
            if (!allCommands.get(key)) {
                unscheduled[i] = key;
                i++;
            }
        }

        return unscheduled;
    }
}
