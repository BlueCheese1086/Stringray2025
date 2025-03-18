package frc.robot;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class AdjustableValues {
    private static HashMap<String,Boolean> hasChanged = new HashMap<String,Boolean>();

    private static HashMap<String,LoggedNetworkBoolean> loggedNetworkBooleans = new HashMap<String,LoggedNetworkBoolean>();
    private static HashMap<String,LoggedNetworkString> loggedNetworkStrings = new HashMap<String,LoggedNetworkString>();
    private static HashMap<String,LoggedNetworkNumber> loggedNetworkNumbers = new HashMap<String,LoggedNetworkNumber>();
    private static HashMap<String,Boolean> loggedBooleans = new HashMap<String,Boolean>();
    private static HashMap<String,String> loggedStrings = new HashMap<String,String>();
    private static HashMap<String,Double> loggedNumbers = new HashMap<String,Double>();

    /**
     * Adds a boolean value to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String shortName, String ntKey, boolean defaultValue) {
        if (loggedBooleans.containsKey(shortName)) return false;

        loggedNetworkBooleans.put(shortName, new LoggedNetworkBoolean(ntKey, defaultValue));
        loggedBooleans.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds a double value to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String shortName, String ntKey, double defaultValue) {
        if (loggedNumbers.containsKey(shortName)) return false;

        loggedNetworkNumbers.put(shortName, new LoggedNetworkNumber(ntKey, defaultValue));
        loggedNumbers.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds a String value to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean register(String shortName, String ntKey, String defaultValue) {
        if (loggedStrings.containsKey(shortName)) return false;

        loggedNetworkStrings.put(shortName, new LoggedNetworkString(ntKey, defaultValue));
        loggedStrings.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns false.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static boolean getBoolean(String shortName) {
        if (!loggedBooleans.containsKey(shortName)) return false;

        hasChanged.put(shortName, false);

        return loggedBooleans.get(shortName);
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns 0.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static double getNumber(String shortName) {
        if (!loggedNumbers.containsKey(shortName)) return 0;

        hasChanged.put(shortName, false);

        return loggedNumbers.get(shortName);
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns 0.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static String getString(String shortName) {
        if (!loggedStrings.containsKey(shortName)) return "";

        hasChanged.put(shortName, false);

        return loggedStrings.get(shortName);
    }

    /**
     * Gets whether or not the value has changed.
     * 
     * If the key has not been created, it returns "".
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static boolean hasChanged(String shortName) {
        if (!hasChanged.containsKey(shortName)) return false;
        
        return hasChanged.get(shortName);
    }

    /** Updates the values with their current value in NetworkTables and updates the corresponding hasChanged value accordingly. */
    public static void updateValues() {
        for (String shortName : loggedNetworkBooleans.keySet()) {
            boolean loggedValue = loggedNetworkBooleans.get(shortName).get();
            if (loggedValue != loggedBooleans.get(shortName)) {
                loggedBooleans.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkNumbers.keySet()) {
            double loggedValue = loggedNetworkNumbers.get(shortName).get();
            if (loggedValue != loggedNumbers.get(shortName)) {
                loggedNumbers.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkStrings.keySet()) {
            String loggedValue = loggedNetworkStrings.get(shortName).get();
            if (loggedValue != loggedStrings.get(shortName)) {
                loggedStrings.put(shortName, loggedValue);
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
        loggedNetworkBooleans.remove(shortName);
        loggedBooleans.remove(shortName);

        loggedNetworkNumbers.remove(shortName);
        loggedNumbers.remove(shortName);

        loggedNetworkStrings.remove(shortName);
        loggedStrings.remove(shortName);

        hasChanged.remove(shortName);
    }
}
