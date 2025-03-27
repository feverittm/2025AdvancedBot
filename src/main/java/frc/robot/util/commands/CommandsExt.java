package frc.robot.util.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;

public class CommandsExt {
    public static Command waitUntilRequirements(BooleanSupplier isFinished, Subsystem... requirements) {
        return new WaitUntilRequirementsCommand(isFinished, requirements);
    }

    public static Command runOnceAndWaitUntil(
            Runnable initialize,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        return new RunOnceWaitUntilCommand(initialize, isFinished, requirements);
    }

    public static Command startIdle(
            Runnable initialize,
            Subsystem... requirements
    ) {
        return new StartIdleCommand(initialize, requirements);
    }

    public static Command startIdleWaitUntil(
            Runnable initialize,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        return new StartIdleWaitUntilCommand(initialize, isFinished, requirements);
    }

    public static Command startEndWaitUntil(
            Runnable initialize,
            Runnable end,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        return new StartEndWaitUntilCommand(initialize, end, isFinished, requirements);
    }

    public static Command schedule(Command... commands) {
        return new ScheduleCommand(commands);
    }

    public static Command onlyIf(BooleanSupplier condition, Command onTrue) {
        return Commands.either(onTrue, Commands.none(), condition);
    }

    public static Command cancelOnTrigger(BooleanSupplier cancelCondition, Command command) {
        return new WrapperCommand(command) {
            @Override
            public boolean isFinished() {
                return cancelCondition.getAsBoolean() || super.isFinished();
            }
        };
    }
}
