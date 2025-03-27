package frc.robot.util.subsystem;

import frc.robot.Robot;

/**
 * A subsystem that isn't handled by the CommandScheduler and thus can't be required.
 * Concept inspired by 6328 but all code is original
 */
public abstract class VirtualSubsystem {
    public VirtualSubsystem() {
        Robot.registerVirtualSubsystem(this);
    }

    public void periodicBeforeCommands() {
    }

    public void periodicAfterCommands() {
    }
}
