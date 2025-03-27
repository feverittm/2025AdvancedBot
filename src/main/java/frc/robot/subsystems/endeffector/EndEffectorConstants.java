package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PIDF;

public class EndEffectorConstants {
    public static final double rollersPositionToleranceRad = Units.degreesToRadians(30);
    public static final double rollersRadiusMeters = Units.inchesToMeters(2.25 / 2.0);

    public static double rollersRadiansForMeters(double meters) {
        return meters / rollersRadiusMeters;
    }

    public static final RollersConfig rollersConfig = new RollersConfig(
            false,
            true,
            40,
            9,
            PIDF.ofP(1),
            switch (Constants.identity) {
                case COMPBOT -> PIDF.ofPSV(0.01, 0.42461, 0.18272);
                case SIMBOT, ALPHABOT -> PIDF.ofSV(0.00995, 0.17859);
            }
    );

    protected static RollersIO createRollersIO() {
        if (Constants.isReplay) {
            return new RollersIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new RollersIOSparkMax(7, rollersConfig);
            case SIMBOT, ALPHABOT -> new RollersIOSim(
                    rollersConfig,
                    0.01,
                    DCMotor.getNEO(1)
            );
        };
    }
}