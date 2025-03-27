//package frc.robot.subsystems.indexer;
//
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotMechanism;
//import frc.robot.RobotState;
//import frc.robot.subsystems.rollers.RollersIO;
//import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
//import frc.robot.util.subsystem.SubsystemBaseExt;
//import lombok.Getter;
//import lombok.RequiredArgsConstructor;
//import org.littletonrobotics.junction.Logger;
//
//import java.util.function.DoubleSupplier;
//
//public class Indexer extends SubsystemBaseExt {
//    private final RobotMechanism robotMechanism = RobotMechanism.get();
//
//    private final RollersIO rollersIO = IndexerConstants.rollersIO;
//    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();
//
//    @RequiredArgsConstructor
//    public enum RollersGoal {
//        CHARACTERIZATION(null),
//        IDLE(() -> 0),
//        INDEX(() -> 1),
//        HANDOFF(() -> 1),
//        EJECT(() -> -1);
//
//        private final DoubleSupplier setpointRadPerSec;
//    }
//
//    @Getter
//    private RollersGoal rollersGoal = RollersGoal.IDLE;
//
//    private static Indexer instance;
//
//    public static Indexer get() {
//        if (instance == null)
//            synchronized (Indexer.class) {
//                instance = new Indexer();
//            }
//
//        return instance;
//    }
//
//    private Indexer() {
//    }
//
//    @Override
//    public void periodicBeforeCommands() {
//        rollersIO.updateInputs(rollersInputs);
//        Logger.processInputs("Inputs/Indexer/Rollers", rollersInputs);
//
//        // TODO: connection alerts
//
//        // side rollers are reversed relative to motor
//        robotMechanism.indexer.rollersLigament.setAngle(Units.radiansToDegrees(-rollersInputs.positionRad));
//    }
//
//    @Override
//    public void periodicAfterCommands() {
//        // TODO: coast override
//        ////////////// ROLLERS //////////////
//        Logger.recordOutput("Indexer/Rollers/Goal", rollersGoal);
//        // TODO: disabled check
//        if (rollersGoal.setpointRadPerSec != null) {
//            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
//            rollersIO.setVelocity(rollersSetpointRadPerSec);
//            Logger.recordOutput("Indexer/Rollers/ClosedLoop", true);
//            Logger.recordOutput("Indexer/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
//        } else {
//            Logger.recordOutput("Indexer/Rollers/ClosedLoop", false);
//        }
//    }
//
//    public Command setGoal(RollersGoal rollersGoal) {
//        return runOnce(() -> this.rollersGoal = rollersGoal);
//    }
//}