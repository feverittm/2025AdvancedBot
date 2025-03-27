package frc.robot.util;

import frc.robot.util.subsystem.SubsystemBaseExt;

import java.util.function.Supplier;

/**
 * A dummy subsystem that can be put in a field of a parent subsystem.
 * <p>
 * Automatically manages cancelling the current command if the goal is changed and allows
 * scheduling commands for the subsystem without interrupting the current goal command.
 */
public class GoalBasedCommandRunner<Goal> extends SubsystemBaseExt {
    private final Supplier<Goal> goalSupplier;
    private Goal lastGoal;

    public GoalBasedCommandRunner(String name, Supplier<Goal> goalSupplier) {
        super(name);
        this.goalSupplier = goalSupplier;
        lastGoal = goalSupplier.get();
    }

    @Override
    public void periodicAfterCommands() {
        var currentGoal = goalSupplier.get();

        // Goal change
        if (currentGoal != lastGoal) {
            // If we have a command scheduled
            var currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                // Cancel it
                currentCommand.cancel();
            }

            lastGoal = currentGoal;
        }
    }
}