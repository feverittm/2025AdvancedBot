package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A dummy subsystem that can be put in a field of a parent subsystem.
 * <p>
 * Allows scheduling commands for the subsystem without interrupting the current goal command.
 * <p>
 * <strong>You MUST run {@link CommandRunner#cancelCommand()} when the goal is changed!</strong>
 * <p>
 * See {@link GoalBasedCommandRunner} for a version of this that will automatically cancel the
 * current command if the goal is changed.
 */
public class CommandRunner extends SubsystemBase {
    public CommandRunner(String name) {
        super(name);
    }

    public void cancelCommand() {
        var currentCommand = getCurrentCommand();
        if (currentCommand != null) {
            currentCommand.cancel();
        }
    }
}