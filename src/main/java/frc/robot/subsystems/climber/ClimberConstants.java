package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
    public static final double currentLimitAmps = 40;
    public static final double gearRatio = 640;

    // 0 = parallel to ground
    public static final double initialPositionRad = Units.degreesToRadians(90);
    public static final double upperLimitRad = Units.degreesToRadians(200);
    public static final double lowerLimitRad = Units.degreesToRadians(-45);

    protected static ClimberIO createIO() {
        if (Constants.isReplay) {
            return new ClimberIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new ClimberIOTalonFX(10, true);
            case SIMBOT, ALPHABOT -> new ClimberIOSim();
        };
    }
}