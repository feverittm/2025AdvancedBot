package frc.robot.util.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/** Combination of Commands.runOnce and WaitUntilRequirements */
/* package-private */ class StartEndWaitUntilCommand extends Command {
    private final Runnable initialize;
    private final Runnable end;
    private final BooleanSupplier isFinished;

    public StartEndWaitUntilCommand(
            Runnable initialize,
            Runnable end,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        this.initialize = initialize;
        this.end = end;
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        initialize.run();
    }

    @Override
    public void end(boolean interrupted) {
        end.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}