package frc.robot;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AdjustableNumbers {
    private static HashMap<String,LoggedNetworkNumber> loggedNetworkNumbers = new HashMap<String,LoggedNetworkNumber>();
    private static HashMap<String,Double> loggedValues = new HashMap<String,Double>();
    private static HashMap<String,Boolean> hasChanged = new HashMap<String,Boolean>();

    public static void register(String key) {
        register(key, key, 0);
    }

    public static void register(String shortName, String ntKey) {
        register(shortName, ntKey, 0);
    }

    public static void register(String shortName, String ntKey, double defaultValue) {
        if (loggedNetworkNumbers.containsKey(shortName)) return;

        loggedNetworkNumbers.put(shortName, new LoggedNetworkNumber(ntKey, defaultValue));
        loggedValues.put(shortName, defaultValue);
        hasChanged.put(shortName, false);
    }

    public static double getValue(String shortName) {
        return loggedValues.get(shortName);
    }

    public static boolean hasChanged(String shortName) {
        return hasChanged.get(shortName);
    }

    public static void updateValues() {
        for (String shortName : loggedNetworkNumbers.keySet()) {
            if (loggedNetworkNumbers.get(shortName).get() != loggedValues.get(shortName)) {
                loggedValues.put(shortName, loggedNetworkNumbers.get(shortName).get());
                hasChanged.put(shortName, true);
            }
        }
    }

    public static void remove(String shortName) {
        loggedNetworkNumbers.remove(shortName);
        loggedValues.remove(shortName);
        hasChanged.remove(shortName);
    }
}
