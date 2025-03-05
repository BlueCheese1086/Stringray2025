package frc.robot;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AdjustableNumbers {
    private static HashMap<String,LoggedNetworkNumber> loggedNetworkNumbers = new HashMap<String,LoggedNetworkNumber>();
    private static HashMap<String,Double> loggedValues = new HashMap<String,Double>();
    private static HashMap<String,Boolean> hasChanged = new HashMap<String,Boolean>();

    /**
     * Adds a value to the logger.
     * The default value is 0.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String key) {
        return register(key, key, 0);
    }

    /**
     * Adds a value to the logged values.
     * The default value is 0.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String shortName, String ntKey) {
        return register(shortName, ntKey, 0);
    }

    /**
     * Adds a value to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String shortName, String ntKey, double defaultValue) {
        if (loggedNetworkNumbers.containsKey(shortName)) return false;

        loggedNetworkNumbers.put(shortName, new LoggedNetworkNumber(ntKey, defaultValue));
        loggedValues.put(shortName, defaultValue);
        hasChanged.put(shortName, false);

        return true;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static double getValue(String shortName) {
        hasChanged.put(shortName, false);

        return loggedValues.get(shortName);
    }

    /**
     * Gets whether or not the value has changed.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static boolean hasChanged(String shortName) {
        return hasChanged.get(shortName);
    }

    /**
     * Updates the values with their current value in NetworkTables and updates the corresponding hasChanged value accordingly.
     */
    public static void updateValues() {
        for (String shortName : loggedNetworkNumbers.keySet()) {
            double loggedValue = loggedNetworkNumbers.get(shortName).get();
            if (loggedValue != loggedValues.get(shortName)) {
                loggedValues.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }
    }

    /**
     * Removes a value from the logger.
     * 
     * @param shortName The value to remove.
     */
    public static void remove(String shortName) {
        loggedNetworkNumbers.remove(shortName);
        loggedValues.remove(shortName);
        hasChanged.remove(shortName);
    }
}
