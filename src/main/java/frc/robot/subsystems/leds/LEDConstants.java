package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LEDConstants {
    public static final int length = 0;

    protected static LEDsIO createIO() {
        if (Constants.isReplay) {
            return new LEDsIO();
        }
        return new LEDsIO();
    }
}