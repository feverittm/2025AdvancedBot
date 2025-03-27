package frc.robot.subsystems.rollers;

import frc.robot.util.PIDF;

public record RollersConfig(
        boolean inverted,
        boolean brakeMode,
        int currentLimit,
        double gearRatio,
        PIDF positionGains,
        PIDF velocityGains
) {
}
