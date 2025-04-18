package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Field;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class AdjustableValues {
    private static HashMap<String,LoggedNetworkBoolean> loggedNetworkBooleans = new HashMap<String,LoggedNetworkBoolean>();
    private static HashMap<String,LoggedNetworkString> loggedNetworkStrings = new HashMap<String,LoggedNetworkString>();
    private static HashMap<String,LoggedNetworkNumber> loggedNetworkNumbers = new HashMap<String,LoggedNetworkNumber>();
    private static HashMap<String,Boolean> loggedBooleans = new HashMap<String,Boolean>();
    private static HashMap<String,String> loggedStrings = new HashMap<String,String>();
    private static HashMap<String,Double> loggedNumbers = new HashMap<String,Double>();
    private static HashMap<String,Boolean> changedValues = new HashMap<String,Boolean>();

    /**
     * Adds a boolean to the logged values.
     * The default return value is false.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerBoolean(String shortName, String ntKey, String... aliases) {
        return registerBoolean(shortName, ntKey, false, aliases);
    }

    /**
     * Adds a boolean to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerBoolean(String shortName, String ntKey, boolean defaultValue, String... aliases) {
        if (changedValues.containsKey(shortName))
            return false;

        LoggedNetworkBoolean loggedBool = new LoggedNetworkBoolean(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (changedValues.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkBooleans.put(aliases[i], loggedBool);
            loggedBooleans.put(aliases[i], defaultValue);
            changedValues.put(aliases[i], true);
        }

        loggedNetworkBooleans.put(shortName, loggedBool);
        loggedBooleans.put(shortName, defaultValue);
        changedValues.put(shortName, true);

        return true;
    }

    public static <T extends StructSerializable> void thing(T val) {
        // new Properties().containsKey(val, "");
        try {
            Field f = val.getClass().getDeclaredField("struct");

            Struct<T> struct = (Struct<T>) f.get(val);

            StructTopic<T> topic = NetworkTableInstance.getDefault().getStructTopic("Thing", struct);
            StructPublisher<T> pub = topic.publish();
            StructSubscriber<T> sub = topic.subscribe(null);
        } catch (NoSuchFieldException | IllegalAccessException err) {
        }

    }

    /**
     * Adds a double to the logged values.
     * The default return value is 0.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerNumber(String shortName, String ntKey, String... aliases) {
        return registerNumber(shortName, ntKey, 0, aliases);
    }

    /**
     * Adds a double to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerNumber(String shortName, String ntKey, double defaultValue, String... aliases) {
        if (changedValues.containsKey(shortName))
            return false;

        LoggedNetworkNumber loggedNum = new LoggedNetworkNumber(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (changedValues.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkNumbers.put(aliases[i], loggedNum);
            loggedNumbers.put(aliases[i], defaultValue);
            changedValues.put(aliases[i], true);
        }

        loggedNetworkNumbers.put(shortName, loggedNum);
        loggedNumbers.put(shortName, defaultValue);
        changedValues.put(shortName, true);

        return true;
    }

    /**
     * Adds a string to the logged values.
     * The default return value is an empty string.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerString(String shortName, String ntKey, String[] aliases) {
        return registerString(shortName, ntKey, "", aliases);
    }

    /**
     * Adds a string to the logged values.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases Alternate keys to read the value with. They have their
     *                own entry in the table and don't affect the read
     *                status of the original short name.
     * 
     * @return Returns false if the short name or any aliases already exist.
     */
    public static boolean registerString(String shortName, String ntKey, String defaultValue, String... aliases) {
        if (changedValues.containsKey(shortName))
            return false;

        LoggedNetworkString loggedStr = new LoggedNetworkString(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (changedValues.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkStrings.put(aliases[i], loggedStr);
            loggedStrings.put(aliases[i], defaultValue);
            changedValues.put(aliases[i], true);
        }

        loggedNetworkStrings.put(shortName, loggedStr);
        loggedStrings.put(shortName, defaultValue);
        changedValues.put(shortName, true);

        return true;
    }

    /**
     * Adds an alias for a short name.
     * 
     * @param shortName The source for the number
     * @param alias The alias to add.
     * 
     * @return Returns false if the short name doesn't exist or if the alias already exists.
     */
    public static boolean addAlias(String shortName, String alias) {
        // Checks if the shortName is a boolean
        if (loggedBooleans.containsKey(shortName) && !loggedBooleans.containsKey(alias)) {
            loggedNetworkBooleans.put(alias, loggedNetworkBooleans.get(shortName));
            loggedBooleans.put(alias, loggedNetworkBooleans.get(shortName).get());
            changedValues.put(alias, true);
            return true;
        }

        // Checks if the shortName is a number
        if (loggedNumbers.containsKey(shortName) && !loggedNumbers.containsKey(alias)) {
            loggedNetworkNumbers.put(alias, loggedNetworkNumbers.get(shortName));
            loggedNumbers.put(alias, loggedNetworkNumbers.get(shortName).get());
            changedValues.put(alias, true);
            return true;
        }

        // Checks if the shortName is a String
        if (loggedStrings.containsKey(shortName) && !loggedStrings.containsKey(alias)) {
            loggedNetworkStrings.put(alias, loggedNetworkStrings.get(shortName));
            loggedStrings.put(alias, loggedNetworkStrings.get(shortName).get());
            changedValues.put(alias, true);
            return true;
        }

        // If the short name doesn't exist or the alias already exists, this runs.
        DriverStation.reportWarning("Program attempted to access an unregistered number, " + shortName + ".", false);
        return false;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * If the key has not been created, it returns false.
     * 
     * @param shortName The short name used to address the boolean.
     */
    public static boolean getBoolean(String shortName) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered boolean, " + shortName + ".", false);
            return false;
        }

        changedValues.put(shortName, false);

        return loggedBooleans.get(shortName);
    }

    /**
     * Sets a value in the logger and marks it as unread.
     * If the key has not been created, it returns false.
     * 
     * @param shortName The short name used to address the boolean.
     * @param value The boolean to push to NT.
     */
    public static boolean setBoolean(String shortName, boolean value) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered boolean, " + shortName + ".", false);
            return false;
        }

        loggedNetworkBooleans.get(shortName).set(value);
        loggedBooleans.put(shortName, value);
        changedValues.put(shortName, true);

        return true;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * If the key has not been created, it returns 0.
     * 
     * @param shortName The short name used to address the boolean.
     */
    public static double getNumber(String shortName) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered number, " + shortName + ".", false);
            return 0;
        }

        changedValues.put(shortName, false);

        return loggedNumbers.get(shortName);
    }

    /**
     * Sets a value in the logger and marks it as unread.
     * If the key has not been created, it returns false.
     * 
     * @param shortName The short name used to address the boolean.
     * @param value The number to push to NT.
     */
    public static boolean setNumber(String shortName, double value) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered number, " + shortName + ".", false);
            return false;
        }

        loggedNetworkNumbers.get(shortName).set(value);
        loggedNumbers.put(shortName, value);
        changedValues.put(shortName, true);

        return true;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * If the key has not been created, it returns an empty string.
     * 
     * @param shortName The short name used to address the boolean.
     */
    public static String getString(String shortName) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered string, " + shortName + ".", false);
            return "";
        }

        changedValues.put(shortName, false);

        return loggedStrings.get(shortName);
    }

    /**
     * Sets a value in the logger and marks it as unread.
     * If the key has not been created, it returns false.
     * 
     * @param shortName The short name used to address the boolean.
     * @param value The string to push to NT.
     */
    public static boolean setString(String shortName, String value) {
        if (!changedValues.containsKey(shortName)) {
            DriverStation.reportWarning("Program attempted to access an unregistered string, " + shortName + ".", false);
            return false;
        }

        loggedNetworkStrings.get(shortName).set(value);
        loggedStrings.put(shortName, value);
        changedValues.put(shortName, true);

        return true;
    }

    /**
     * Gets whether or not a value has changed.
     * If the key has not been created, it returns false.
     * 
     * @param shortName The short name used to address data.
     */
    public static boolean hasChanged(String shortName) {
        if (!changedValues.containsKey(shortName))
            return false;

        return changedValues.get(shortName);
    }

    /** Updates the values with their current value in NetworkTables and marks changed things as unread. */
    public static void updateValues() {
        for (String shortName : loggedNetworkBooleans.keySet()) {
            boolean loggedValue = loggedNetworkBooleans.get(shortName).get();
            if (loggedValue != loggedBooleans.get(shortName)) {
                loggedBooleans.put(shortName, loggedValue);
                changedValues.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkNumbers.keySet()) {
            double loggedValue = loggedNetworkNumbers.get(shortName).get();
            if (loggedValue != loggedNumbers.get(shortName)) {
                loggedNumbers.put(shortName, loggedValue);
                changedValues.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkStrings.keySet()) {
            String loggedValue = loggedNetworkStrings.get(shortName).get();
            if (loggedValue != loggedStrings.get(shortName)) {
                loggedStrings.put(shortName, loggedValue);
                changedValues.put(shortName, true);
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

        changedValues.remove(shortName);
    }
}