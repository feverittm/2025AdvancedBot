package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.rollersConfig;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = rollersConfig.positionGains().tunable("EndEffector/Position");
    public static final PIDF.Tunable velocityGainsTunable = rollersConfig.velocityGains().tunable("EndEffector/Velocity");

    public static final LoggedTunableNumber funnelIntakeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/FunnelIntake", Units.rotationsPerMinuteToRadiansPerSecond(500));
    public static final LoggedTunableNumber scoreCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoral", Units.rotationsPerMinuteToRadiansPerSecond(300));
    public static final LoggedTunableNumber descoreAlgaeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/DescoreAlgae", Units.rotationsPerMinuteToRadiansPerSecond(-600));
    public static final LoggedTunableNumber ejectGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/Eject", Units.rotationsPerMinuteToRadiansPerSecond(450));
}