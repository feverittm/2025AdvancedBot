package frc.robot.util.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/** Like WaitUntil, but requires a subsystem and doesn't run while disabled. */
/* package-private */ class WaitUntilRequirementsCommand extends Command {
    private final BooleanSupplier isFinished;

    public WaitUntilRequirementsCommand(BooleanSupplier isFinished, Subsystem... requirements) {
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}