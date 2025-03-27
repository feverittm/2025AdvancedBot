package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.AutoAlignLocations.*;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureTuning.funnelIntakeFinalizeInches;
import static frc.robot.util.HighFrequencySamplingThread.highFrequencyLock;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Drive drive = Drive.get();
    //    private final CoralIntake coralIntake = CoralIntake.get();
//    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    public enum Goal {
        IDLE,

        INTAKE_CORAL_WAIT_PIVOT,
        INTAKE_CORAL_INTAKING,
        INDEXING_PIVOT_DOWN,
        INDEXING_PIVOT_UP,
        HANDOFF_WAIT_ELEVATOR,
        HANDOFF_HANDING_OFF,

        MANUAL_SCORE_CORAL_WAIT_ELEVATOR,
        MANUAL_SCORE_CORAL_WAIT_CONFIRM,
        MANUAL_SCORE_CORAL_SCORING,

        AUTO_SCORE_CORAL_WAIT_INITIAL,
        AUTO_SCORE_CORAL_WAIT_INITIAL_RAISING,
        AUTO_SCORE_CORAL_WAIT_FINAL,
        AUTO_SCORE_CORAL_WAIT_ELEVATOR,
        AUTO_SCORE_CORAL_SCORING,

        DESCORE_ALGAE_WAIT_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        FUNNEL_INTAKE_WAITING,
        FUNNEL_INTAKE_FINALIZING,

        AUTO_FUNNEL_INTAKE_WAITING_ALIGN,
        AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
        AUTO_FUNNEL_INTAKE_FINALIZING,

        EJECT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final Debouncer endEffectorBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer endEffectorBeamBreakDebouncerLong = new Debouncer(0.25);

    private Command withGoal(Goal goal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                Superstructure.this.goal = goal;
                super.initialize();
            }
        };
    }

    private Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
    }

    @Override
    public void periodicBeforeCommands() {
        highFrequencyLock.lock();

        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);

        highFrequencyLock.unlock();

//        robotMechanism.coralIntake.rangeLigament.setColor(
//                intakeRangeTriggered()
//                        ? new Color8Bit(Color.kGreen)
//                        : new Color8Bit(Color.kRed)
//        );
//        robotMechanism.indexer.beamBreakLigament.setColor(
//                inputs.indexerBeamBreakTriggered
//                        ? new Color8Bit(Color.kGreen)
//                        : new Color8Bit(Color.kRed)
//        );
        robotMechanism.endEffector.beamBreakLigament.setColor(
                endEffectorTriggeredShort() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

//    private boolean intakeRangeTriggered() {
//        return inputs.intakeRangeMeters <= intakeRangeTriggerMeters;
//    }

    /** Reacts quickly to change so better for waiting for the beam break */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredShort")
    private boolean endEffectorTriggeredShort() {
        return endEffectorBeamBreakDebouncerShort.calculate(inputs.endEffectorBeamBreakTriggered);
    }

    /** Reacts slowly to change so better for gating commands */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredLong")
    private boolean endEffectorTriggeredLong() {
        return endEffectorBeamBreakDebouncerLong.calculate(inputs.endEffectorBeamBreakTriggered);
    }

//    public Command waitUntilIntakeTriggered() {
//        // This should not require the superstructure because we don't want to conflict with setGoal
//        return Commands.waitUntil(this::intakeRangeTriggered);
//    }
//
//    public Command waitUntilIndexerTriggered() {
//        // This should not require the superstructure because we don't want to conflict with setGoal
//        return Commands.waitUntil(() -> inputs.indexerBeamBreakTriggered);
//    }

    public Command waitUntilEndEffectorTriggered(Command ifIgnored) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.either(
                ifIgnored,
                Commands.waitUntil(this::endEffectorTriggeredShort),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command waitUntilEndEffectorNotTriggered(Command ifIngored) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.either(
                ifIngored,
                Commands.waitUntil(() -> !endEffectorTriggeredShort()),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command idle() {
        return setGoal(Goal.IDLE).andThen(Commands.idle());
    }

//    public Command coralIntakeIdle() {
//        return coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE).andThen(Commands.idle());
//    }

//    public Command indexerIdle() {
//        return indexer.setGoal(Indexer.RollersGoal.IDLE).andThen(Commands.idle());
//    }

    public Command elevatorIdle() {
        return elevator.setGoal(() -> Elevator.Goal.STOW).andThen(Commands.idle());
    }

    public Command endEffectorIdle() {
        return endEffector.setGoal(EndEffector.RollersGoal.IDLE).andThen(Commands.idle());
    }

//    public Command intakeCoral() {
//        return Commands.sequence(
//                Commands.parallel(
//                        setGoal(Goal.INTAKE_CORAL_WAIT_PIVOT),
//                        coralIntake.setGoalsAndWaitUntilAtPivotGoal(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.IDLE)
//                ),
//                Commands.parallel(
//                        setGoal(Goal.INTAKE_CORAL_INTAKING),
//                        coralIntake.setGoals(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.INTAKE),
//                        indexer.setGoal(Indexer.RollersGoal.INDEX)
//                ),
//                waitUntilIntakeTriggered(),
//                // Branch off into an uncancelable sequence to prevent indexing being messed up
//                CommandsExt.schedule(
//                        Commands.sequence(
//                                // Wait at least a small amount of time, or until we are done indexing to bring the intake up
//                                Commands.parallel(
//                                        setGoal(Goal.INDEXING_PIVOT_DOWN),
//                                        Commands.race(
//                                                Commands.waitSeconds(0.25),
//                                                waitUntilIndexerTriggered()
//                                        )
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.INDEXING_PIVOT_UP),
//                                        coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
//                                        waitUntilIndexerTriggered()
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.HANDOFF_WAIT_ELEVATOR),
//                                        indexer.setGoal(Indexer.RollersGoal.IDLE),
//                                        elevator.setGoalAndWaitUntilAtGoal(() -> Elevator.Goal.STOW)
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.HANDOFF_HANDING_OFF),
//                                        indexer.setGoal(Indexer.RollersGoal.HANDOFF),
//                                        endEffector.setGoal(EndEffector.RollersGoal.HANDOFF),
//                                        waitUntilEndEffectorTriggered()
//                                )
//                                // TODO: move forward X radians
//                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
//                )
//        );
//    }

    public Command eject() {
        return Commands.parallel(
                setGoal(Goal.EJECT),
                endEffector.setGoal(EndEffector.RollersGoal.EJECT),
                Commands.idle()
        );
    }

    public Command scoreCoralManual(
            boolean duringAuto,
            BooleanSupplier forwardCondition,
            BooleanSupplier cancelCondition,
            Supplier<Elevator.Goal> elevatorGoalSupplier
    ) {
        Command raiseElevator = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_ELEVATOR),
                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
        );
        Command waitConfirm = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_CONFIRM),
                Commands.waitUntil(forwardCondition)
        );
        Command score = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_SCORING),
                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                elevator.setGoal(elevatorGoalSupplier),
                duringAuto
                        ? Commands.none()
                        : waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle
        Command finalize = Commands.waitSeconds(scoreCoralSettleSeconds);
        if (duringAuto) {
            return Commands.sequence(raiseElevator, waitConfirm, score, finalize);
        } else {
            Command cmd = Commands.sequence(
                    raiseElevator,
                    waitConfirm,
                    // Don't allow canceling
                    CommandsExt.schedule(score.andThen(finalize).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
            );
            return CommandsExt.onlyIf(
                    () -> endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    CommandsExt.cancelOnTrigger(
                            cancelCondition,
                            cmd
                    )
            );
        }
    }

    public Command descoreAlgaeManual(Supplier<Elevator.Goal> elevatorGoalSupplier) {
        Command cmd = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.DESCORE_ALGAE_WAIT_ELEVATOR),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
                ),
                Commands.parallel(
                        setGoal(Goal.DESCORE_ALGAE_DESCORING),
                        endEffector.setGoal(EndEffector.RollersGoal.DESCORE_ALGAE),
                        elevator.setGoal(elevatorGoalSupplier),
                        Commands.idle()
                )
        );
        return CommandsExt.onlyIf(
                () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                cmd
        );
    }

    public Command funnelIntake(boolean duringAuto) {
        Command intake = Commands.deadline(
                waitUntilEndEffectorTriggered(Commands.idle()),
                setGoal(Goal.FUNNEL_INTAKE_WAITING),
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE)
        );
        Command finalize = Commands.parallel(
                setGoal(Goal.FUNNEL_INTAKE_FINALIZING),
                endEffector.moveByAndWaitUntilDone(() -> Units.inchesToMeters(funnelIntakeFinalizeInches.get()))
        );
        if (duringAuto) {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(intake, finalize)
            );
        } else {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            // Don't allow canceling
                            CommandsExt.schedule(finalize.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                    )
            );
        }
    }

    public Command funnelIntakeWithAutoAlign(boolean duringAuto, Station station) {
        Command autoAlign = drive.moveTo(station.alignPoseSupplier);
        Command intake = Commands.deadline(
                waitUntilEndEffectorTriggered(Commands.idle()),
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                Commands.sequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN),
                                autoAlign.until(() -> isAtPoseWithTolerance(
                                        station.alignPoseSupplier.get(),
                                        stationAlignToleranceXYMeters,
                                        stationAlignToleranceOmegaRad
                                ))
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE),
                                drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                                        ? new ChassisSpeeds(-0.1, -0.1, -0.2)
                                        : new ChassisSpeeds(0.1, 0.1, 0.2))
                        )
                )
        );
        Command finalize = Commands.parallel(
                setGoal(Goal.AUTO_FUNNEL_INTAKE_FINALIZING),
                endEffector.moveByAndWaitUntilDone(() -> Units.inchesToMeters(funnelIntakeFinalizeInches.get()))
        );
        if (duringAuto) {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(intake, finalize)
            );
        } else {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            // Don't allow canceling
                            CommandsExt.schedule(finalize.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                    )
            );
        }
    }

    private boolean isAtPoseWithTolerance(Pose2d desiredPose, double linearToleranceMeters, double angularToleranceRad) {
        Pose2d currentPose = robotState.getPose();
        return desiredPose.getTranslation().getDistance(currentPose.getTranslation()) < linearToleranceMeters
                && Math.abs(desiredPose.getRotation().minus(currentPose.getRotation()).getRadians()) < angularToleranceRad;
    }

    public Command autoAlignAndScore(
            boolean duringAuto,
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            BooleanSupplier forceCondition,
            BooleanSupplier cancelCondition
    ) {
        Command initial = Commands.race(
                // Drive to initial position
                drive.moveTo(() -> getInitialAlignPose(reefSideSupplier.get(), sideSupplier.get())),
                Commands.sequence(
                        // Wait until we are within elevator raise tolerance
                        Commands.parallel(
                                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_INITIAL),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(() -> Elevator.Goal.STOW),
                                Commands.waitUntil(() -> isAtPoseWithTolerance(
                                        getInitialAlignPose(reefSideSupplier.get(), sideSupplier.get()),
                                        initialElevatorRaiseToleranceMeters,
                                        initialAlignToleranceRad
                                )) // We don't care about velocity at this point
                        ),
                        // Start raising elevator and wait until we are within align tolerance
                        Commands.parallel(
                                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_INITIAL_RAISING),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(elevatorGoalSupplier),
                                Commands.waitUntil(() ->
                                        isAtPoseWithTolerance(
                                                getInitialAlignPose(reefSideSupplier.get(), sideSupplier.get()),
                                                initialAlignToleranceMeters,
                                                initialAlignToleranceRad
                                        )
                                                && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < initialAlignToleranceRadPerSecond
                                )
                        ),
                        Commands.waitSeconds(0.3)
                )
        );
        DoubleSupplier elevatorPercentageSupplier = () -> operatorDashboard.disableInterpolateAutoAlign.get()
                ? 1
                : elevator.getPositionMeters() / elevatorGoalSupplier.get().setpointMeters.getAsDouble();
        Supplier<Command> driveFinal = () -> drive.moveTo(() -> getFinalAlignPose(elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get()));
        Command waitFinalAndElevator = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FINAL),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(elevatorGoalSupplier),
                        Commands.waitUntil(() ->
                                isAtPoseWithTolerance(
                                        getFinalAlignPose(elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get()),
                                        finalAlignToleranceMeters,
                                        finalAlignToleranceRad
                                )
                                        && Math.abs(drive.getMeasuredChassisLinearVelocityMetersPerSec()) < finalAlignToleranceMetersPerSecond
                                        && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < finalAlignToleranceRadPerSecond
                        )
                ),
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_ELEVATOR),
                        elevator.waitUntilAtGoal()
                ),
                Commands.waitSeconds(0.3)
        );
        // Don't allow forcing for a bit, then check if force is true
        Command waitForForce = Commands.waitSeconds(2).andThen(Commands.waitUntil(forceCondition));
        Command score = Commands.parallel(
                setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.parallel(
                elevator.setGoal(() -> Elevator.Goal.STOW),
                Commands.waitSeconds(scoreCoralSettleSeconds)
        );
        if (duringAuto) {
            return Commands.sequence(
                    initial,
                    Commands.race(
                            driveFinal.get(),
                            Commands.sequence(
                                    Commands.race(
                                            waitFinalAndElevator,
                                            waitForForce
                                    ),
                                    score,
                                    finalize
                            )
                    )
            );
        } else {
            return CommandsExt.onlyIf(
                    // Only run if you have coral and are in front of your reef side
                    () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                            && alignable(reefSideSupplier.get(), RobotState.get().getPose()),
                    CommandsExt.cancelOnTrigger(
                            cancelCondition,
                            Commands.sequence(
                                    initial,
                                    Commands.race(
                                            driveFinal.get(),
                                            waitFinalAndElevator,
                                            waitForForce
                                    ),
                                    // don't allow cancelling
                                    CommandsExt.schedule(Commands.race(
                                            driveFinal.get(),
                                            score.andThen(finalize)
                                    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                            )
                    )
            );
        }
    }
}