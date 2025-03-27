package frc.robot.subsystems.elevator;

import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("Elevator/Gains");

    public static final LoggedTunableNumber maxVelocityMetersPerSecondTunable =
            new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSecond", maxVelocityMetersPerSecond);
    public static final LoggedTunableNumber maxAccelerationMetersPerSecondSquaredTunable =
            new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSecondSquared", maxAccelerationMetersPerSecondSquared);

    public static final LoggedTunableNumber stowGoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/Stow", 0);
    public static final LoggedTunableNumber scoreL1GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL1", 0.23);
    public static final LoggedTunableNumber scoreL2GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL2", 0.7);
    public static final LoggedTunableNumber scoreL3GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL3", 1.1);
    public static final LoggedTunableNumber scoreL4GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL4", maxHeightMeters);
    public static final LoggedTunableNumber descoreL2GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/DescoreL2", 0.6);
    public static final LoggedTunableNumber descoreL3GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/DescoreL3", 1);
}