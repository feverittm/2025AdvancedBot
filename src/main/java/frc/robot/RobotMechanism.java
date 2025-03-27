package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

/** Holds the Mechanism2d and all roots and ligaments that visualizes the robot state */
public class RobotMechanism {
    private static RobotMechanism instance;

    public static RobotMechanism get() {
        if (instance == null)
            synchronized (RobotMechanism.class) {
                instance = new RobotMechanism();
            }

        return instance;
    }

    private RobotMechanism() {
    }

    /** Middle of the robot in the mechanism */
    public static final double middleOfRobot = 0.75;

    @AutoLogOutput(key = "RobotState/Mechanism")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 2.1, new Color8Bit(Color.kBlack));

    public final Elevator elevator = new Elevator();
    public final EndEffector endEffector = new EndEffector();
    public final Climber climber = new Climber();
//    public final Indexer indexer = new Indexer();
//    public final CoralIntake coralIntake = new CoralIntake();

    public class Elevator {
        public final LoggedMechanismRoot2d stage1Root = mechanism.getRoot("elevator_stage1", 0, 0);
        public final LoggedMechanismRoot2d stage2Root = mechanism.getRoot("elevator_stage2", 0, 0);
        public final LoggedMechanismRoot2d stage3Root = mechanism.getRoot("elevator_stage3", 0, 0);
        private final LoggedMechanismRoot2d hardstopSlowdownRoot = mechanism.getRoot("elevator_hardstopSlowdown", 0, 0);

        public void updateHardstopSlowdownPosition() {
            hardstopSlowdownRoot.setPosition(middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + hardstopSlowdownMeters);
        }

        private Elevator() {
            var baseRoot = mechanism.getRoot(
                    "elevatorBase",
                    middleOfRobot - Units.inchesToMeters(7) + 0.06,
                    Units.inchesToMeters(1.85)
            );
            baseRoot.append(new LoggedMechanismLigament2d(
                    "base",
                    Units.inchesToMeters(33.2),
                    90,
                    13,
                    new Color8Bit(new Color(0.2, 0.2, 0.2))
            ));

            stage1Root.append(new LoggedMechanismLigament2d(
                    "stage1",
                    Units.inchesToMeters(32.5),
                    90,
                    12,
                    new Color8Bit(new Color(0.3, 0.3, 0.3))
            ));

            stage2Root.append(new LoggedMechanismLigament2d(
                    "stage2",
                    Units.inchesToMeters(32),
                    90,
                    11,
                    new Color8Bit(new Color(0.4, 0.4, 0.4))
            ));

            stage3Root.append(new LoggedMechanismLigament2d(
                    "stage3",
                    Units.inchesToMeters(7),
                    90,
                    10,
                    new Color8Bit(new Color(0.5, 0.5, 0.5))
            ));

            var hardstopRoot = mechanism.getRoot("elevator_hardstop", middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + hardstopMeters);
            hardstopRoot.append(new LoggedMechanismLigament2d(
                    "hardstop",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kGray)
            ));

            updateHardstopSlowdownPosition();
            hardstopSlowdownRoot.append(new LoggedMechanismLigament2d(
                    "hardstopSlowdown",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kYellow)
            ));

            var autoStopUpperRoot = mechanism.getRoot("elevator_autoStopUpper", middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + upperLimit.positionMeters());
            autoStopUpperRoot.append(new LoggedMechanismLigament2d(
                    "autoStopUpper",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kRed)
            ));

            var autoStopLowerRoot = mechanism.getRoot("elevator_autoStopLower", middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + lowerLimit.positionMeters());
            autoStopLowerRoot.append(new LoggedMechanismLigament2d(
                    "autoStopLower",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kRed)
            ));
        }
    }

    public class EndEffector {
        public final LoggedMechanismRoot2d root = mechanism.getRoot("endEffector", 0, 0);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(10),
                90,
                10,
                new Color8Bit(Color.kPurple)
        ));

        public final LoggedMechanismRoot2d beamBreakRoot = mechanism.getRoot("endEffector_beamBreak", 0, 0);
        public final LoggedMechanismLigament2d beamBreakLigament = beamBreakRoot.append(new LoggedMechanismLigament2d(
                "beamBreak",
                Units.inchesToMeters(1),
                0,
                11,
                new Color8Bit(Color.kRed)
        ));

        public final LoggedMechanismRoot2d topRollersRoot = mechanism.getRoot("endEffector_topRollers", 0, 0);
        public final LoggedMechanismLigament2d topRollersLigament = topRollersRoot.append(new LoggedMechanismLigament2d(
                "endEffector_topRollers",
                Units.inchesToMeters(1),
                0,
                12,
                new Color8Bit(Color.kOrange)
        ));

        private EndEffector() {
        }
    }

    public class Climber {
        public final LoggedMechanismRoot2d root = mechanism.getRoot("climber", middleOfRobot, 0.5);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(15),
                0,
                5,
                new Color8Bit(Color.kCyan)
        ));

        private Climber() {

        }
    }

//    public class Indexer {
//        private static final double x = middleOfRobot + Units.inchesToMeters(12.2);
//        private static final double y = Units.inchesToMeters(6);
//        private static final double angle = 180 - 13.815;
//
//        public final LoggedMechanismRoot2d root = mechanism.getRoot("indexer", x, y + 0.065);
//
//        public final LoggedMechanismRoot2d beamBreakRoot = mechanism.getRoot(
//                "indexer_beamBreak",
//                x - Units.inchesToMeters(9.5),
//                y + Units.inchesToMeters(5)
//        );
//        public final LoggedMechanismLigament2d beamBreakLigament = beamBreakRoot.append(new LoggedMechanismLigament2d(
//                "indexer_beamBreak",
//                Units.inchesToMeters(1),
//                angle,
//                11,
//                new Color8Bit(Color.kRed)
//        ));
//
//        public final LoggedMechanismRoot2d rollersRoot = mechanism.getRoot(
//                "indexer_rollers",
//                x - Units.inchesToMeters(5),
//                y + Units.inchesToMeters(7)
//        );
//        public final LoggedMechanismLigament2d rollersLigament = rollersRoot.append(new LoggedMechanismLigament2d(
//                "indexer_rollers",
//                Units.inchesToMeters(1),
//                0,
//                12,
//                new Color8Bit(Color.kOrange)
//        ));
//
//        private Indexer() {
//            root.append(new LoggedMechanismLigament2d(
//                    "ligament",
//                    Units.inchesToMeters(14.75),
//                    angle,
//                    10,
//                    new Color8Bit(Color.kBlue)
//            ));
//        }
//    }
//
//    public class CoralIntake {
//        public final LoggedMechanismRoot2d root = mechanism.getRoot("coralIntake", 0, 0);
//        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
//                "ligament",
//                Units.inchesToMeters(1), // width, x
//                0,
//                35, // height, y
//                new Color8Bit(Color.kGreen)
//        ));
//
//        public final LoggedMechanismRoot2d rangeRoot = mechanism.getRoot(
//                "coralIntake_range",
//                middleOfRobot + Units.inchesToMeters(18),
//                Units.inchesToMeters(7)
//        );
//        public final LoggedMechanismLigament2d rangeLigament = rangeRoot.append(new LoggedMechanismLigament2d(
//                "coralIntake_range",
//                Units.inchesToMeters(1),
//                135,
//                11,
//                new Color8Bit(Color.kRed)
//        ));
//
//        public final LoggedMechanismRoot2d topRollersRoot = mechanism.getRoot(
//                "coralIntake_topRollers",
//                middleOfRobot + Units.inchesToMeters(23),
//                Units.inchesToMeters(7)
//        );
//        public final LoggedMechanismLigament2d topRollersLigament = topRollersRoot.append(new LoggedMechanismLigament2d(
//                "coralIntake_topRollers",
//                Units.inchesToMeters(1),
//                0,
//                12,
//                new Color8Bit(Color.kOrange)
//        ));
//
//        private CoralIntake() {
//        }
//    }
}
