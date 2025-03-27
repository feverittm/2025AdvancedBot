package frc.robot.util.network;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.HashSet;

/** Note: automatically adds <code>/Tuning</code> to key */
public class LoggedNetworkBooleanExt extends LoggedNetworkBoolean {
    private final HashSet<Integer> hashCodesGottenChange = new HashSet<>();
    private boolean lastValue;
    private boolean hasChangedOnce = false;

    public LoggedNetworkBooleanExt(String key, boolean defaultValue) {
        super("/Tuning/" + removeSlash(key), defaultValue);
        lastValue = defaultValue;
    }

    public boolean hasChanged(int hashCode) {
        if (hasChangedOnce && !hashCodesGottenChange.contains(hashCode)) {
            hashCodesGottenChange.add(hashCode);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        boolean newValue = get();
        if (newValue != lastValue) {
            // Clear all hash codes that have been given the change so that they get the new value
            hashCodesGottenChange.clear();
            hasChangedOnce = true;
        }
        lastValue = newValue;
    }
}
