
package frc.robot.util;

import java.util.HashMap;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;

public class AdjustableValues {
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("Logged");

    private static HashMap<String,Publisher> pubs = new HashMap<String,Publisher>();
    private static HashMap<String,Subscriber> subs = new HashMap<String,Subscriber>();
    private static HashMap<String,Long> lastReads = new HashMap<String,Long>();

    // Logger functions

    /**
     * Logs a boolean array to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The boolean array to log.
     */
    public static void log(String key, boolean[] val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof BooleanArrayPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to boolean array.", false);

        // Creating the topic and storing the publishers and subscribers.
        BooleanArrayTopic topic = table.getBooleanArrayTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a boolean to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The boolean to log.
     */
    public static void log(String key, boolean val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof BooleanPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to boolean.", false);

        // Creating the topic and storing the publishers and subscribers.
        BooleanTopic topic = table.getBooleanTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a double array to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The double array to log.
     */
    public static void log(String key, double[] val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof DoubleArrayPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to double array.", false);

        // Creating the topic and storing the publishers and subscribers.
        DoubleArrayTopic topic = table.getDoubleArrayTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a double to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The double to log.
     */
    public static void log(String key, double val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof DoublePublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to double.", false);

        // Creating the topic and storing the publishers and subscribers.
        DoubleTopic topic = table.getDoubleTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a float array to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The float array to log.
     */
    public static void log(String key, float[] val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof FloatArrayPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to float array.", false);

        // Creating the topic and storing the publishers and subscribers.
        FloatArrayTopic topic = table.getFloatArrayTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a float to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The float to log.
     */
    public static void log(String key, float val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof FloatPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to float.", false);

        // Creating the topic and storing the publishers and subscribers.
        FloatTopic topic = table.getFloatTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs an int array to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The int array to log.
     */
    public static void log(String key, int[] val) {
        // Converting the int array to a long array
        long[] longs = new long[val.length];
        for (int i = 0; i < val.length; i++) {
            longs[i] = val[i];
        }

        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof IntegerArrayPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(longs);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to integer array.", false);

        // Creating the topic and storing the publishers and subscribers.
        IntegerArrayTopic topic = table.getIntegerArrayTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(longs));
    }

    /**
     * Logs an int to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The int to log.
     */
    public static void log(String key, int val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof IntegerPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to integer.", false);

        // Creating the topic and storing the publishers and subscribers.
        IntegerTopic topic = table.getIntegerTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a String array to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The String array to log.
     */
    public static void log(String key, String[] val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof StringArrayPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to String array.", false);

        // Creating the topic and storing the publishers and subscribers.
        StringArrayTopic topic = table.getStringArrayTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    /**
     * Logs a String to NetworkTables.
     *
     * @param key The key to log the value under.
     * @param val The String to log.
     */
    public static void log(String key, String val) {
        // If the publisher already exists under the same type, log the value and exit.
        if (pubs.get(key) instanceof StringPublisher pub) {
            lastReads.put(key, System.currentTimeMillis());
            pub.set(val);
            return;
        }

        // Reporting if the logged data type is being changed.
        // This runs if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to String.", false);

        // Creating the topic and storing the publishers and subscribers.
        StringTopic topic = table.getStringTopic(key);
        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(val));
    }

    public static <T extends StructSerializable> void log(String key, T value) {
        // If the pub and sub is a StructPublisher and StructSubscriber respectively,
        // and the topic type is the same as the value parameter then this updates its
        // value.
        if (pubs.get(key) instanceof StructPublisher pub && subs.get(key) instanceof StructSubscriber sub) {
            if (sub.get().getClass().getName().equals(value.getClass().getName())) {
                ((StructPublisher<T>) pub).set(value);
                return;
            }
        }

        // Reporting if the logged data type is being changed.
        // This triggers if the publisher exists.
        if (pubs.containsKey(key))
            DriverStation.reportWarning("Switching type of key \"" + key + "\" to " + value.getClass().getName() + ".", false);

        // If the pub and sub aren't StructPublishers or StructSubscribers, or the topic
        // type is not the same as the value parameter, then it creates a new topic.
        // This also runs if the pub and sub don't exist.
        Struct<T> struct = null;

        try {
            struct = (Struct<T>) value.getClass().getDeclaredField("struct").get(value);
        } catch (IllegalAccessException | NoSuchFieldException err) {
            DriverStation.reportError(
                    "No public instance of struct for the StructSerializable object " + value.getClass().getName(),
                    err.getStackTrace());
            return;
        }

        StructTopic<T> topic = table.getStructTopic(key, struct);

        pubs.put(key, topic.publish());
        subs.put(key, topic.subscribe(value));

        lastReads.put(key, System.currentTimeMillis());
    }

    // Getter Functions

    /**
     * Gets a boolean array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static boolean[] get(String key, boolean[] defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof BooleanArraySubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a boolean from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static boolean get(String key, boolean defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof BooleanSubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a double array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static double[] get(String key, double[] defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof DoubleArraySubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a double from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static double get(String key, double defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof DoubleSubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a float array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static float[] get(String key, float[] defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof FloatArraySubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a float from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static float get(String key, float defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof FloatSubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets an int array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static int[] get(String key, int[] defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof IntegerArraySubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());

            long[] longArr = sub.get();
            int[] intArr = new int[longArr.length];

            for (int i = 0; i < longArr.length; i++) {
                intArr[i] = (int) longArr[i];
            }

            return intArr;
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets an int from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static int get(String key, int defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof IntegerSubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return (int) sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a String array from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static String[] get(String key, String[] defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StringArraySubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a String from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static String get(String key, String defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StringSubscriber sub) {
            lastReads.put(key, System.currentTimeMillis());
            return sub.get();
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    /**
     * Gets a struct serialized object from NetworkTables.
     *
     * @param key The key to find the value under.
     * @param defaultVal The value to return if the subscriber doesn't exist.
     */
    public static <T extends StructSerializable> T get(String key, T defaultVal) {
        // If the subscriber already exists under the same type, return the value.
        if (subs.get(key) instanceof StructSubscriber sub) {
            if (sub.get().getClass().getName().equals(defaultVal.getClass().getName())) {
                lastReads.put(key, System.currentTimeMillis());
                return (T) sub.get();
            }
        }

        // Reporting if the logged data type is being changed or the publisher doesn't exist.
        if (pubs.containsKey(key)) {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Topic exists, but is the wrong type.", false);
        } else {
            DriverStation.reportWarning("Error retrieving value \"" + key + "\".  Subscriber does not exist.", false);
        }

        return defaultVal;
    }

    public static boolean hasChanged(String key) {
        return subs.get(key).getLastChange() > lastReads.get(key);
    }

    public static void remove(String key) {
        pubs.get(key).close();
        pubs.remove(key);

        subs.get(key).close();
        subs.remove(key);

        lastReads.remove(key);
    }
}
