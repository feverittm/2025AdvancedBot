package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    private final RollersIO rollersIO = createRollersIO();
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        HANDOFF(() -> 0),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get),
        SCORE_CORAL(scoreCoralGoalSetpoint::get),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get),
        EJECT(ejectGoalSetpoint::get),
        GO_TO_POSITION(null); // Handled specially in periodic and with rollersPositionSetpointRad

        private final DoubleSupplier setpointRadPerSec;
    }

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private Double rollersPositionSetpointRad = null;

    private final Alert rollersDisconnectedAlert = new Alert("End effector rollers motor is disconnected.", Alert.AlertType.kError);

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }

    private EndEffector() {
    }

    @Override
    public void periodicBeforeCommands() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/EndEffector/Rollers", rollersInputs);

        rollersDisconnectedAlert.set(!rollersInputs.connected);

        robotMechanism.endEffector.ligament.setAngle(getAngleDegrees());
        robotMechanism.endEffector.ligament.setAngle(getAngleDegrees());
        // top rollers are reversed relative to motor
        robotMechanism.endEffector.topRollersLigament.setAngle(Units.radiansToDegrees(-rollersInputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            rollersIO.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(hashCode(), rollersIO::setPositionPIDF);
        velocityGainsTunable.ifChanged(hashCode(), rollersIO::setVelocityPIDF);

        ////////////// ROLLERS //////////////
        Logger.recordOutput("EndEffector/Rollers/Goal", rollersGoal);
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
            rollersIO.setOpenLoop(0);
        } else if (rollersGoal.setpointRadPerSec != null) {
            // Velocity control
            var rollersVelocitySetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersVelocitySetpointRadPerSec);
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/Velocity/SetpointRadPerSec", rollersVelocitySetpointRadPerSec);
        } else if (rollersGoal == RollersGoal.GO_TO_POSITION && rollersPositionSetpointRad != null) {
            // Position control
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Position/SetpointRad", rollersPositionSetpointRad);
            rollersIO.setPosition(rollersPositionSetpointRad);
        } else {
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
        }
    }

    public Command setGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }

    public boolean atPositionSetpoint() {
        return Math.abs(rollersInputs.positionRad - rollersPositionSetpointRad) <= rollersPositionToleranceRad;
    }

    /** Goes positionDeltaMeters forward (or backwards) from current position */
    public Command moveByAndWaitUntilDone(DoubleSupplier positionDeltaMeters) {
        return startEndWaitUntil(
                () -> {
                    this.rollersGoal = RollersGoal.GO_TO_POSITION;
                    rollersPositionSetpointRad = rollersInputs.positionRad + rollersRadiansForMeters(positionDeltaMeters.getAsDouble());
                },
                () -> {
                    this.rollersGoal = RollersGoal.IDLE;
                    rollersPositionSetpointRad = null;
                },
                this::atPositionSetpoint
        );
    }

    public double getAngleDegrees() {
        return MathUtil.clamp(
                // After 5 inches, interpolate to 40 degrees finishing at 7.25 inches
                90 + (40 / Units.inchesToMeters(2.25) * (elevator.getPositionMeters() - Units.inchesToMeters(5))),
                90, 130
        );
    }

    public Command rollersFeedforwardCharacterization() {
        return setGoal(RollersGoal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        rollersIO::setOpenLoop,
                        () -> new double[]{rollersInputs.velocityRadPerSec},
                        1,
                        this
                ));
    }
}