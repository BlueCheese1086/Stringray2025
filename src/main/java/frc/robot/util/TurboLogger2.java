package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class TurboLogger2 {
    // Hashmaps for NT logging
    private static HashMap<String, Publisher> pubs = new HashMap<String, Publisher>();
    private static HashMap<String, Subscriber> subs = new HashMap<String, Subscriber>();
    private static HashMap<String, Long> lastReads = new HashMap<String, Long>();
    private static HashMap<String, List<String>> ntPathToAliases = new HashMap<String, List<String>>();
    private static HashMap<String, String> aliasToNTPath = new HashMap<String, String>();

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("TurboLogger");

    /**
     * Enables DataLog recording of NT output.
     *
     * <p>The wpilog is created at the logPath point. If the logPath ends with a forward slash, it
     * is seen as a directory. Otherwise, it is seen as a file.
     *
     * @param logPath The path to store the logfile at.
     */
    public static void enableDataLogs(String logPath) {
        // Checking if the path contains a file.
        // If there is a period in the path, it is considered to be a file.
        if (logPath.contains(".")) {

            // Checking if the path is separated by forwards or backwards slashes.
            String separator = "";
            if (logPath.contains("/")) {
                separator = "/";
            } else if (logPath.contains("\\")) {
                separator = "\\";
            }

            // Splitting the path and putting it back together without the file name.
            String path = "";
            String[] pathParts = logPath.split(separator);
            for (int i = 0; i < pathParts.length - 1; i++) {
                path += pathParts[i] + separator;
            }

            // Starting the logger at the path and file location.
            DataLogManager.start(path, pathParts[pathParts.length - 1]);
        } else {
            DataLogManager.start(logPath, "");
        }

        DataLogManager.logNetworkTables(true);
    }

    /** Closes the existing DataLog file and resets it to null. */
    public static void disableDataLogs() {
        // Stopping the logger from pulling from NetworkTables.
        DataLogManager.logNetworkTables(false);
    }

    // Error messages
    public static void pubsubTypeMismatch(String key, String type, boolean ispub) {
        String pubsub = ispub ? "Publisher" : "Subscriber";
        if (aliasToNTPath.containsKey(key)) {
            DriverStation.reportWarning(pubsub + " is not an instance of " + type + pubsub + " for alias \"" + key + "\" of key \"" + aliasToNTPath.get(key) + "\".", false);
        } else {
            DriverStation.reportWarning(pubsub + " is not an instance of " + type + pubsub + " for key \"" + key + "\".", false);
        }
    }

    public static void noSub(String key) {
        if (aliasToNTPath.containsKey(key)) {
            DriverStation.reportWarning("Subscriber does not exist for alias \"" + key + "\" of key \"" + aliasToNTPath.get(key) + "\".", false);
        } else {
            DriverStation.reportWarning("Subscriber does not exist for key \"" + key + "\".", false);
        }
    }

    public static void structTypeMismatch(String key, String desiredType) {
        if (aliasToNTPath.containsKey(key)) {
            DriverStation.reportWarning("Value is not an instance of " + desiredType + " for alias \"" + key + "\" of key \"" + aliasToNTPath.get(key) + "\".", false);
        } else {
            DriverStation.reportWarning("Value is not an instance of " + desiredType + " for key \"" + key + "\".", false);
        }
    }

    // Loggers

    /**
     * Logs a boolean array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The boolean array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, boolean[] value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a boolean array, then it pushes the value.
            if (pubs.get(key) instanceof BooleanArrayPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "BooleanArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the boolean array topic
        BooleanArrayTopic topic = table.getBooleanArrayTopic(key);

        BooleanArrayPublisher pub = topic.publish();
        BooleanArraySubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a boolean to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The boolean to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, boolean value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published to NT already.
        if (pubs.containsKey(key)) {
            // If the published value is a boolean, then it pushes the value and exits.
            if (pubs.get(key) instanceof BooleanPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "Boolean", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the boolean topic
        BooleanTopic topic = table.getBooleanTopic(key);

        BooleanPublisher pub = topic.publish();
        BooleanSubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a double array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The double array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, double[] value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a double array, then it pushes the value and exits.
            if (pubs.get(key) instanceof DoubleArrayPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "DoubleArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the double array topic
        DoubleArrayTopic topic = table.getDoubleArrayTopic(key);

        DoubleArrayPublisher pub = topic.publish();
        DoubleArraySubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a double to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The double to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, double value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a double, then it pushes the value and exits.
            if (pubs.get(key) instanceof DoublePublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "Double", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the double topic
        DoubleTopic topic = table.getDoubleTopic(key);

        DoublePublisher pub = topic.publish();
        DoubleSubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a float array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The float array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, float[] value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a float array, then it pushes the value and exits.
            if (pubs.get(key) instanceof FloatArrayPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "FloatArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the float array topic
        FloatArrayTopic topic = table.getFloatArrayTopic(key);

        FloatArrayPublisher pub = topic.publish();
        FloatArraySubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a float to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The float to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, float value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a float, then it pushes the value and exits.
            if (pubs.get(key) instanceof FloatPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "Float", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the float topic
        FloatTopic topic = table.getFloatTopic(key);

        FloatPublisher pub = topic.publish();
        FloatSubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs an int array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The int array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, int[] value, String... aliases) {
        // Converting the int array to a long array.
        long[] longArr = new long[value.length];
        for (int i = 0; i < value.length; i++) {
            longArr[i] = value[i];
        }

        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is an int array, then it pushes the value and exits.
            if (pubs.get(key) instanceof IntegerArrayPublisher pub) {
                pub.set(longArr);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "IntegerArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the int array topic
        IntegerArrayTopic topic = table.getIntegerArrayTopic(key);

        IntegerArrayPublisher pub = topic.publish();
        IntegerArraySubscriber sub = topic.subscribe(longArr);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs an int to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The int to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, int value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is an int, then it pushes the value and exits.
            if (pubs.get(key) instanceof IntegerPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "Integer", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the int topic
        IntegerTopic topic = table.getIntegerTopic(key);

        IntegerPublisher pub = topic.publish();
        IntegerSubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a string array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The string array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, String[] value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a string array, then it pushes the value and exits.
            if (pubs.get(key) instanceof StringArrayPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "StringArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the string array topic
        StringArrayTopic topic = table.getStringArrayTopic(key);

        StringArrayPublisher pub = topic.publish();
        StringArraySubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a string to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The string to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static void log(String key, String value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet.
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a string, then it pushes the value.
            if (pubs.get(key) instanceof StringPublisher pub) {
                pub.set(value);
                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "String", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the string topic
        StringTopic topic = table.getStringTopic(key);

        StringPublisher pub = topic.publish();
        StringSubscriber sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a struct array to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The struct array to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static <T extends StructSerializable> void log(String key, T[] value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Finding the struct for this StructSerializable object.
        Struct<T> struct = null;

        try {
            struct = (Struct<T>) value.getClass().getComponentType().getDeclaredField("struct").get(value);
        } catch (IllegalAccessException | NoSuchFieldException err) {
            DriverStation.reportError("No public instance of struct for the StructSerializable object " + value.getClass().getName(), err.getStackTrace());
            return;
        }

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a struct array, then it pushes the value and exits.
            if (pubs.get(key) instanceof StructArrayPublisher pub && subs.get(key) instanceof StructArraySubscriber sub) {
                if (sub.get().getClass().getComponentType().getName().equals(value.getClass().getComponentType().getName())) {
                    ((StructArrayPublisher<T>) pub).set(value);
                    return;
                }

                // Reports if the struct array being pushed doesn't match the type of the existing
                // struct array.
                // If the key is an alias, then change the message reported to DriverStation.
                structTypeMismatch(key, sub.get().getClass().getName());

                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "StructArray", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the struct array topic
        StructArrayTopic<T> topic = table.getStructArrayTopic(key, struct);

        StructArrayPublisher<T> pub = topic.publish();
        StructArraySubscriber<T> sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    /**
     * Logs a struct to NetworkTables.
     *
     * <p>It also creates any of the aliases passed into the array.
     *
     * @param key The key to log the value under. This can be a NetworkTables path or an alias.
     * @param value The struct to log.
     * @param aliases Any aliases to add to the ntPath.
     */
    public static <T extends StructSerializable> void log(String key, T value, String... aliases) {
        // Adding any aliases for the NT path.
        if (aliases.length > 0) addAliases(key, aliases);

        // Adding the value to the lastReads map if it isn't there yet
        if (!lastReads.containsKey(key)) lastReads.put(key, 0l);

        // Finding the struct for this StructSerializable object.
        Struct<T> struct = null;

        try {
            struct = (Struct<T>) value.getClass().getDeclaredField("struct").get(value);
        } catch (IllegalAccessException | NoSuchFieldException err) {
            DriverStation.reportError("No public instance of struct for the StructSerializable object " + value.getClass().getName(), err.getStackTrace());
            return;
        }

        // Checking if the key has been published already.
        if (pubs.containsKey(key)) {
            // If the published value is a struct, then it pushes the value and exits.
            if (pubs.get(key) instanceof StructPublisher pub && subs.get(key) instanceof StructSubscriber sub) {
                if (sub.get().getClass().getName().equals(value.getClass().getName())) {
                    ((StructPublisher<T>) pub).set(value);
                    return;
                }

                // Reports if the struct being pushed doesn't match the existing struct
                // If the key is an alias, then change the message reported to DriverStation.
                structTypeMismatch(key, sub.get().getClass().getName());

                return;
            }

            // Reporting if the publisher type doesn't match up.
            // If the key is an alias, then change the message reported to DriverStation.
            pubsubTypeMismatch(key, "Struct", true);

            return;
        }

        // Since the publisher for this key hasn't been created, it makes one.
        // If the key has any aliases defined, it also pushes the pubs and subs to them.
        // Creating the struct topic
        StructTopic<T> topic = table.getStructTopic(key, struct);

        StructPublisher<T> pub = topic.publish();
        StructSubscriber<T> sub = topic.subscribe(value);

        pubs.put(key, pub);
        subs.put(key, sub);

        // This updates the pub/sub/lastread entries for all the aliases of this path.
        for (String alias : ntPathToAliases.getOrDefault(key, new ArrayList<>())) {
            pubs.put(alias, pub);
            subs.put(alias, sub);
            lastReads.put(alias, 0l);
        }
    }

    // Getters

    /**
     * Gets a boolean array from NetworkTables.
     *
     * By default, it will return an empty boolean array.
     * 
     * @param key The key to find the value under.
     */
    public static boolean[] getBooleanArray(String key) {
        return getBooleanArray(key, new boolean[0]);
    }

    /**
     * Gets a boolean array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static boolean[] getBooleanArray(String key, boolean[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof BooleanArraySubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "BooleanArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a boolean from NetworkTables.
     *
     * It returns false by default.
     * 
     * @param key The key to find the value under.
     */
    public static boolean getBoolean(String key) {
        return getBoolean(key, false);
    }

    /**
     * Gets a boolean from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static boolean getBoolean(String key, boolean defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof BooleanSubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "Boolean", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a double array from NetworkTables.
     *
     * It returns an empty double array by default.
     * 
     * @param key The key to find the value under.
     */
    public static double[] getDoubleArray(String key) {
        return getDoubleArray(key, new double[0]);
    }

    /**
     * Gets a double array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static double[] getDoubleArray(String key, double[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof DoubleArraySubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "DoubleArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a double from NetworkTables.
     *
     * It returns 0 by default.
     * 
     * @param key The key to find the value under.
     */
    public static double getDouble(String key) {
        return getDouble(key, 0);
    }

    /**
     * Gets a double from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static double getDouble(String key, double defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof DoubleSubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "Double", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a float array from NetworkTables.
     *
     * It returns an empty float array by default.
     * 
     * @param key The key to find the value under.
     */
    public static float[] getFloatArray(String key) {
        return getFloatArray(key, new float[0]);
    }

    /**
     * Gets a float array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static float[] getFloatArray(String key, float[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof FloatArraySubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "FloatArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a float from NetworkTables.
     *
     * It returns 0 by default.
     * 
     * @param key The key to find the value under.
     */
    public static float getFloat(String key) {
        return getFloat(key, 0);
    }

    /**
     * Gets a float from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static float getFloat(String key, float defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof FloatSubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "Float", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets an int array from NetworkTables.
     *
     * It returns an empty integer array by default.
     * 
     * @param key The key to find the value under.
     */
    public static int[] getIntegerArray(String key) {
        return getIntegerArray(key, new int[0]);
    }

    /**
     * Gets an int array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static int[] getIntegerArray(String key, int[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof IntegerArraySubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            // Converting the long array to an int array.
            // It clamps the int values rather than cause them to overflow.
            long[] longArr = sub.get();
            int[] intArr = new int[longArr.length];

            for (int i = 0; i < longArr.length; i++) {
                if (longArr[i] > Integer.MAX_VALUE) intArr[i] = Integer.MAX_VALUE;
                if (longArr[i] < Integer.MIN_VALUE) intArr[i] = Integer.MIN_VALUE;
                else intArr[i] = (int) longArr[i];
            }

            return intArr;
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "IntegerArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets an int from NetworkTables.
     *
     * It returns 0 by default.
     * 
     * @param key The key to find the value under.
     */
    public static int getInteger(String key) {
        return getInteger(key, 0);
    }

    /**
     * Gets an int from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static int getInteger(String key, int defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof IntegerSubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            // Converting the long to an int.
            // It clamps the int values rather than cause them to overflow.
            long longVal = sub.get();

            if (longVal > Integer.MAX_VALUE) return Integer.MAX_VALUE;
            if (longVal < Integer.MIN_VALUE) return Integer.MIN_VALUE;
            else return (int) longVal;
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "Integer", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a string array from NetworkTables.
     *
     * It returns an empty string array by default.
     * 
     * @param key The key to find the value under.
     */
    public static String[] getStringArray(String key) {
        return getStringArray(key, new String[0]);
    }

    /**
     * Gets a string array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static String[] getStringArray(String key, String[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StringArraySubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "StringArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets a string from NetworkTables.
     *
     * It returns an empty string by default.
     * 
     * @param key The key to find the value under.
     */
    public static String getString(String key) {
        return getString(key, "");
    }

    /**
     * Gets a string from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static String getString(String key, String defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StringSubscriber sub) {
            lastReads.put(key, sub.getLastChange());

            return sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "String", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Gets an array of struct serialized objects from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static <T extends StructSerializable> T[] getStructArray(String key, T[] defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StructArraySubscriber sub) {
            if (sub.get().getClass().getComponentType().getName().equals(defaultValue.getClass().getComponentType().getName())) {
                lastReads.put(key, sub.getLastChange());
                return (T[]) sub.get();
            }

            // Reporting if the class of the StructArraySubscriber doesn't match the class passed in
            // as the defaultValue.
            structTypeMismatch(key, sub.get().getClass().getName());

            return defaultValue;
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "StructArray", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    public static <T extends StructSerializable> T getStruct(String key) {
        return getStruct(key, null);
    }

    /**
     * Gets a struct serialized object from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultValue The value to return if the subscriber doesn't exist.
     */
    public static <T extends StructSerializable> T getStruct(String key, T defaultValue) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StructSubscriber sub) {
            // Trying to return the sub, but if the types don't match, then it will throw an error.
            return (T) sub.get();
        }

        // Reporting if the logged data type is different or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            pubsubTypeMismatch(key, "Struct", false);
        } else {
            noSub(key);
        }

        return defaultValue;
    }

    /**
     * Adds aliases to a NT path. Aliases allow you to pass them into the log or get methods rather
     * than the full path. They can also increase readability in the code.
     *
     * <p>Note that aliases have their own entry in the lastRead table, so when you get a value with
     * one alias, it does not mark itself as read for any others.
     *
     * @param ntPath The path to create an alias for.
     * @param aliases The aliases to add.
     */
    public static void addAliases(String ntPath, String... aliases) {
        // This check isn't necessary, but I want it to report a warning if someone tried it.
        if (aliases.length == 0) {
            DriverStation.reportWarning(
                    "Please don't use addAliases with no alias parameters", false);
            return;
        }

        for (String alias : aliases) {
            // Skipping if the alias is the same as the path.
            if (alias.equals(ntPath)) {
                DriverStation.reportWarning(
                        "Alias cannot have the same name as the NT path.  Skipping creation",
                        false);
                continue;
            }

            // If the alias has already been assigned, it reports an error and doesn't add an entry
            // for
            // this ntPath.
            if (aliasToNTPath.containsKey(alias)) {
                DriverStation.reportWarning(
                        "Alias \""
                                + alias
                                + "\" has already been assigned to key \""
                                + aliasToNTPath.get(alias)
                                + "\".  Skipping creation",
                        false);
                continue;
            }

            // Recording the alias in the aliasToNTKey table.
            aliasToNTPath.put(alias, ntPath);

            // Adding the alias to the ntPathToAliases table.
            // If the list doesn't exist yet, it creates one.
            if (!ntPathToAliases.containsKey(ntPath)) {
                ntPathToAliases.put(ntPath, new ArrayList<String>());
            }

            ntPathToAliases.get(ntPath).add(alias);

            // Now that the aliases have been added to the records, let's put them into the
            // publisher/subscriber maps.
            // This only adds the publishers and subscribers if both exist for the ntPath.
            if (pubs.containsKey(ntPath) && subs.containsKey(ntPath)) {
                pubs.put(alias, pubs.get(ntPath));
                subs.put(alias, subs.get(ntPath));
                lastReads.put(alias, lastReads.get(ntPath));
            }
        }
    }

    /**
     * Gets whether or not the logged value has changed since the last time the key was read from.
     *
     * @param key The key to check the status of. This can be the path in NetworkTables or an alias.
     */
    public static boolean hasChanged(String key) {
        System.out.println(lastReads.containsKey(key));
        System.out.println(lastReads.get(key));
        System.out.println(subs.get(key).getLastChange());

        return lastReads.containsKey(key) && (lastReads.get(key) < subs.get(key).getLastChange());
    }

    /**
     * Removes a key from the logger.
     *
     * <p>If the key is a NT path, then it removes its aliases. If the key is an alias itself, it
     * removes the alias.
     *
     * @param key
     */
    public static void remove(String key) {
        pubs.get(key).close();
        pubs.remove(key);

        subs.get(key).close();
        subs.remove(key);

        lastReads.remove(key);

        // Removing the key and its aliases from the maps.
        if (ntPathToAliases.containsKey(key)) {
            for (String alias : ntPathToAliases.get(key)) {
                aliasToNTPath.remove(alias);
            }

            ntPathToAliases.remove(key);
        }

        // Removing the alias from the alias maps.
        if (aliasToNTPath.containsKey(key)) {
            ntPathToAliases.get(aliasToNTPath.get(key)).remove(key);

            aliasToNTPath.remove(key);
        }
    }
}
