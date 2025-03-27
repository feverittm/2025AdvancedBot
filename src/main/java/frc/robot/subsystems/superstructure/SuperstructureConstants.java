package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureConstants {
    /** Range for the intake sensor to be triggered */
    public static final double intakeRangeTriggerMeters = 1;

    public static final double scoreCoralSettleSeconds = 0.25;

    protected static SuperstructureIO createIO() {
        if (Constants.isReplay) {
            return new SuperstructureIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new SuperstructureIOReal();
            case SIMBOT, ALPHABOT -> new SuperstructureIOSim();
        };
    }
}