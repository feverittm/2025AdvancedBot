package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.AutoAlignLocations;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.commands.CommandsExt;

public class BargeSideAuto {
    public static Command get(AutoRoutine routine) {
        final var superstructure = Superstructure.get();

        final var firstScoreTraj = routine.trajectory("Barge Side", 0);
        final var secondStationTraj = routine.trajectory("Barge Side", 1);
        final var secondScoreTraj = routine.trajectory("Barge Side", 2);
        final var thirdStationTraj = routine.trajectory("Barge Side", 3);
        final var thirdScoreTraj = routine.trajectory("Barge Side", 4);

        var ref = new Object() {
            boolean isFinished = false;
        };

        routine.active().onTrue(
                Commands.sequence(
                        firstScoreTraj.resetOdometry(),
                        firstScoreTraj.cmd()
                )
        );

        firstScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftBack,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false
                ),
                CommandsExt.schedule(secondStationTraj.cmd()) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        ));

        secondStationTraj.atTime("intake").onTrue(Commands.sequence(
                superstructure.funnelIntakeWithAutoAlign(true, AutoAlignLocations.Station.BargeSide),
                secondScoreTraj.cmd()
        ));
        secondScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftFront,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false
                ),
                CommandsExt.schedule(thirdStationTraj.cmd()) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        ));

        thirdStationTraj.atTime("intake").onTrue(Commands.sequence(
                superstructure.funnelIntakeWithAutoAlign(true, AutoAlignLocations.Station.BargeSide),
                thirdScoreTraj.cmd()
        ));
        thirdScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftFront,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false

                ),
                Commands.runOnce(() -> ref.isFinished = true)
        ));

        return routine.cmd(() -> ref.isFinished).beforeStarting(() -> ref.isFinished = false);
    }
}
