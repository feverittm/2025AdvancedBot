package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final Translation2d[] stationLocations = {
            new Translation2d(1, 1),
            new Translation2d(1, 7),
            new Translation2d(16.5, 7),
            new Translation2d(16.5, 1)
    };

    private final IntakeSimulation intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Coral",
            ModuleIOSim.driveSimulation,
            // Width of the intake
            Meters.of(driveConfig.trackWidthMeters()),
            // The extension length of the intake beyond the robot's frame (when activated)
            Inches.of(10),
            IntakeSimulation.IntakeSide.FRONT,
            1
    );

    private final RobotState robotState = RobotState.get();
    //    private final CoralIntake coralIntake = CoralIntake.get();
//    private final Indexer indexer = Indexer.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Elevator elevator = Elevator.get();

    private final Timer sinceCoralIntaked = new Timer();
    private static final double indexTime = 1;
    private final Timer sinceStartedHandoff = new Timer();
    private static final double handoffTime = 0.5;
    private final Timer sinceAtStation = new Timer();
    private static final double stationIntakeTime = 1.5;
    private CoralState coralState = CoralState.IN_END_EFFECTOR; // preload

    private enum CoralState {
        NO_CORAL,
        INDEXING,
        IN_INDEXER,
        HANDING_OFF,
        IN_END_EFFECTOR
    }

    public SuperstructureIOSim() {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> coralState = CoralState.IN_END_EFFECTOR));
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
//        if (coralIntake.getRollersGoal() == CoralIntake.RollersGoal.INTAKE && coralState == CoralState.NO_CORAL) {
//            intakeSimulation.startIntake();
//        } else {
//            intakeSimulation.stopIntake();
//        }

        var intakedCoral = intakeSimulation.getGamePiecesAmount() > 0;
        if (intakedCoral && coralState == CoralState.NO_CORAL) {
            coralState = CoralState.INDEXING;
            intakeSimulation.obtainGamePieceFromIntake();
            sinceCoralIntaked.restart();
        }

        var pose = robotState.getPose();
        Transform3d coralRobotRelative = null;
        switch (coralState) {
            case NO_CORAL -> {
                var current = robotState.getPose().getTranslation();
                if (Arrays.stream(stationLocations).anyMatch(t -> t.getDistance(current) < 0.7) && endEffector.getRollersGoal() == EndEffector.RollersGoal.FUNNEL_INTAKE) {
                    if (!sinceAtStation.isRunning()) sinceAtStation.restart();
                    if (sinceAtStation.hasElapsed(stationIntakeTime))
                        coralState = CoralState.IN_END_EFFECTOR;
                } else {
                    sinceAtStation.stop();
                }
            }
            case INDEXING -> {
                if (sinceCoralIntaked.hasElapsed(indexTime)) {
                    coralState = CoralState.IN_INDEXER;
                }
                var interp = MathUtil.clamp(sinceCoralIntaked.get() / indexTime, 0, 1);
                coralRobotRelative = new Transform3d(
                        Units.inchesToMeters(20) - Units.inchesToMeters(15) * interp,
                        0,
                        Units.inchesToMeters(7) + Units.inchesToMeters(4) * interp,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(30),
                                MathUtil.clamp(
                                        Units.degreesToRadians(80) - Units.degreesToRadians(90) * interp,
                                        0,
                                        90
                                )
                        )
                );
            }
            case IN_INDEXER -> {
//                if (indexer.getRollersGoal() == Indexer.RollersGoal.HANDOFF) {
//                    coralState = CoralState.HANDING_OFF;
//                    sinceStartedHandoff.restart();
//                }
                coralRobotRelative = new Transform3d(
                        Units.inchesToMeters(5),
                        0,
                        Units.inchesToMeters(11),
                        new Rotation3d(0, Units.degreesToRadians(13), 0)
                );
            }
            case HANDING_OFF -> {
                if (sinceStartedHandoff.hasElapsed(handoffTime)) {
                    coralState = CoralState.IN_END_EFFECTOR;
                }
                var interp = MathUtil.clamp(sinceStartedHandoff.get() / handoffTime, 0, 1);
                coralRobotRelative = new Transform3d(
                        Units.inchesToMeters(5) - Units.inchesToMeters(9) * interp,
                        0,
                        Units.inchesToMeters(11) + Units.inchesToMeters(1) * interp,
                        new Rotation3d(0, Units.degreesToRadians(7), 0)
                );
            }
            case IN_END_EFFECTOR -> {
                var angle = Units.degreesToRadians(-endEffector.getAngleDegrees() - 90);
                var coralOffsetX = Units.inchesToMeters(-8.5) + Units.inchesToMeters(6) * Math.tan(angle);
                // TODO: fix the trig, it doesn't actually work but is good enough for sim
                var coralOffsetZ = Units.inchesToMeters(13.5) + elevator.getPositionMeters() + Units.inchesToMeters(4) * Math.tan(angle);
                if (endEffector.getRollersGoal() == EndEffector.RollersGoal.SCORE_CORAL || endEffector.getRollersGoal() == EndEffector.RollersGoal.EJECT) {
                    coralState = CoralState.NO_CORAL;
                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                    pose.getTranslation(),
                                    new Translation2d(coralOffsetX - Units.inchesToMeters(2), 0),
                                    ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    pose.getRotation(),
                                    // The height at which the coral is ejected
                                    Meters.of(coralOffsetZ + Units.inchesToMeters(2)),
                                    // The initial speed of the coral
                                    MetersPerSecond.of(-1),
                                    Degrees.of(65)
                            ));
                } else {
                    coralRobotRelative = new Transform3d(
                            coralOffsetX,
                            0,
                            coralOffsetZ,
                            new Rotation3d(0, angle, 0)
                    );
                }
            }
        }
        if (coralRobotRelative != null) {
            Logger.recordOutput("FieldSimulation/CoralInRobot", new Pose3d[]{
                    new Pose3d(
                            pose.getX(),
                            pose.getY(),
                            0,
                            new Rotation3d(0, 0, pose.getRotation().getRadians())
                    ).transformBy(coralRobotRelative)
            });
        } else {
            Logger.recordOutput("FieldSimulation/CoralInRobot", new Pose3d[]{});
        }

        switch (coralState) {
//            case INDEXING -> {
//                inputs.intakeRangeMeters = 0;
//                inputs.indexerBeamBreakTriggered = false;
//                inputs.endEffectorBeamBreakTriggered = false;
//            }
//            case IN_INDEXER, HANDING_OFF -> {
//                inputs.intakeRangeMeters = Double.MAX_VALUE;
//                inputs.indexerBeamBreakTriggered = true;
//                inputs.endEffectorBeamBreakTriggered = false;
//            }
            case IN_END_EFFECTOR -> {
//                inputs.intakeRangeMeters = Double.MAX_VALUE;
//                inputs.indexerBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = true;
            }
            default -> {
//                inputs.intakeRangeMeters = Double.MAX_VALUE;
//                inputs.indexerBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = false;
            }
        }
    }
}
