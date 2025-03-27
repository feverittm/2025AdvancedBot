package frc.robot.util.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Commands.startEnd without the end */
/* package-private */ class StartIdleCommand extends Command {
    private final Runnable initialize;

    public StartIdleCommand(
            Runnable initialize,
            Subsystem... requirements
    ) {
        this.initialize = initialize;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        initialize.run();
    }
}