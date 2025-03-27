//package frc.robot.subsystems.coralintake;
//
//import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.util.Units;
//import frc.robot.Constants;
//import frc.robot.subsystems.rollers.RollersConfig;
//import frc.robot.subsystems.rollers.RollersIO;
//import frc.robot.subsystems.rollers.RollersIOSim;
//import frc.robot.util.PIDF;
//
//public class CoralIntakeConstants {
//    public static final double pivotLengthMeters = Units.inchesToMeters(16);
//    public static final double pivotSetpointToleranceRad = Units.degreesToRadians(15);
//    public static final double pivotMaxVelocityRadPerSec = Units.degreesToRadians(500);
//    public static final double pivotMaxAccelerationRadPerSecSquared = Units.degreesToRadians(1500);
//
//    protected static RollersIO createRollersIO() {}
//    protected static final RollersIO rollersIo = Constants.isReplay
//            ? new RollersIO()
//            : switch (Constants.identity) {
//        case COMPBOT -> null;
//        case SIMBOT, ALPHABOT -> new RollersIOSim(
//                new RollersConfig(
//                        false,
//                        true,
//                        40,
//                        3,
//                        PIDF.ofP(1),
//                        PIDF.ofPSVA(1, 0, 1, 1)
//                ),
//                0.01,
//                DCMotor.getNEO(1)
//        );
//    };
//
//    public static final PivotConfig pivotConfig = switch (Constants.identity) {
//        case COMPBOT -> new PivotConfig(
//                // TODO: Tune PID
//                PIDF.ofPIDSVAG(1, 0.0, 0.0, 0, 0, 0, 0),
//                60,
//                false,
//                // TODO: Figure this out
//                false,
//                40
//        );
//        case SIMBOT, ALPHABOT -> new PivotConfig(
//                PIDF.ofPSVAG(0.2, 0, 1.2, 1.2, 1.2),
//                60,
//                false,
//                // TODO: Figure this out
//                false,
//                40
//        );
//    };
//
//    protected static PivotIO createPivotIO() {}
//    protected static final PivotIO pivotIo = Constants.isReplay
//            ? new PivotIO()
//            : switch (Constants.identity) {
//        case COMPBOT -> null;
//        case SIMBOT, ALPHABOT -> new PivotIOSim();
//    };
//
//    public record PivotConfig(
//            PIDF gains,
//            double motorGearRatio,
//            boolean motorInverted,
//            boolean encoderInverted,
//            double currentLimit
//    ) {
//    }
//}
