package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.factories.auto.BargeSideAuto;
import frc.robot.factories.auto.ProcessorSideAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.subsystem.VirtualSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

import static frc.robot.Constants.mode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends VirtualSubsystem {
    // Controller
    private final CommandXboxController driverController = RobotBase.isSimulation()
            ? Constants.Simulation.simController.apply(0)
            : new CommandXboxController(0);
    private final Alert driverControllerDisconnectedAlert = new Alert("Driver controller is not connected!", Alert.AlertType.kError);

    // Dashboard inputs
    /** THERE SHOULD NOT BE A DEFAULT OPTION OR THE ALERT WILL BREAK!! */
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");
    private final Alert autoNotChosenAlert = new Alert("Auto is not chosen!", Alert.AlertType.kError);

    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    /* Subsystems */
    // Note: order does matter
    private final Elevator elevator = Elevator.get();
    //    private final CoralIntake coralIntake = CoralIntake.get();
//    private final Indexer indexer = Indexer.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Vision vision = Vision.get();
    private final Drive drive = Drive.get();
    private final Superstructure superstructure = Superstructure.get();
    private final LEDs leds = LEDs.get();
    private final Climber climber = Climber.get();

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        setDefaultCommands();
        configureButtonBindings();
    }

    private void addAutos() {
        final var factory = drive.createAutoFactory();

        // THERE SHOULD NOT BE A DEFAULT OPTION OR THE ALERT WILL BREAK!!

        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("Barge Side", BargeSideAuto.get(factory.newRoutine("Barge Side")));
        autoChooser.addOption("Processor Side", ProcessorSideAuto.get(factory.newRoutine("Processor Side")));
        autoChooser.addOption("Leave", drive.runRobotRelative(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(5));

        autoChooser.addOption("Characterization", Commands.deferredProxy(characterizationChooser::get));
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption("Drive Feedforward Characterization", drive.feedforwardCharacterization());
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysId.dynamic(SysIdRoutine.Direction.kReverse));

        ////////////////////// ELEVATOR //////////////////////

        characterizationChooser.addOption("Elevator SysId (Quasistatic Forward)", elevator.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Elevator SysId (Quasistatic Reverse)", elevator.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Elevator SysId (Dynamic Forward)", elevator.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Elevator SysId (Dynamic Reverse)", elevator.sysId.dynamic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Elevator Feedforward Characterization", elevator.feedforwardCharacterization());

        ////////////////////// END EFFECTOR //////////////////////

        characterizationChooser.addOption("End Effector Rollers Feedforward Characterization", endEffector.rollersFeedforwardCharacterization());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.driveJoystick(
                        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
                        // forward on joystick is negative y - we want positive x for forward
                        () -> -driverController.getLeftY(),
                        // right on joystick is positive x - we want negative y for right
                        () -> -driverController.getLeftX(),
                        // right on joystick is positive x - we want negative x for right (CCW is positive)
                        () -> -driverController.getRightX(),
                        () -> {
//                            if (coralIntake.getRollersGoal() != CoralIntake.RollersGoal.INTAKE)
                            return Optional.empty();
//                            var gamepiece = vision.getClosestGamepiece();
//                            return gamepiece.map(gamepieceTranslation -> {
//                                var relativeToRobot = gamepieceTranslation.minus(robotState.getTranslation());
//                                if (relativeToRobot.getNorm() < Units.feetToMeters(1)) {
//                                    // Don't try to face towards it if we are too close
//                                    return new Pose2d(gamepieceTranslation, robotState.getRotation());
//                                } else {
//                                    // Try to face towards the game piece
//                                    var toGamepiece = new Rotation2d(relativeToRobot.getX(), relativeToRobot.getY());
//                                    return new Pose2d(gamepieceTranslation, toGamepiece);
//                                }
//                            });
                        }
                )
        );

        superstructure.setDefaultCommand(superstructure.idle().ignoringDisable(true));
//        coralIntake.setDefaultCommand(superstructure.coralIntakeIdle().ignoringDisable(true));
//        indexer.setDefaultCommand(superstructure.indexerIdle().ignoringDisable(true));
        elevator.setDefaultCommand(superstructure.elevatorIdle().ignoringDisable(true));
        endEffector.setDefaultCommand(superstructure.endEffectorIdle().ignoringDisable(true));
        climber.setDefaultCommand(climber.idle());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverController.y().onTrue(robotState.resetRotation());

        driverController.x().whileTrue(superstructure.eject());

        driverController.rightTrigger().whileTrue(superstructure.funnelIntake(false));
//        driverController.rightTrigger().whileTrue(superstructure.intakeCoral());

        driverController.leftTrigger().onTrue(Commands.either(
                superstructure.scoreCoralManual(
                        false,
                        driverController.leftTrigger(),
                        driverController.leftBumper(),
                        operatorDashboard::getCoralScoringElevatorGoal
                ).asProxy(),
                superstructure.autoAlignAndScore(
                        false,
                        operatorDashboard::getSelectedReefZoneSide,
                        operatorDashboard::getSelectedLocalReefSide,
                        operatorDashboard::getCoralScoringElevatorGoal,
                        driverController.leftTrigger(),
                        driverController.leftBumper()
                ).asProxy(),
                // Use manual scoring if override enabled or when scoring L1
                () -> operatorDashboard.manualScoring.get()
                        || operatorDashboard.getSelectedCoralScoringLevel() == OperatorDashboard.CoralScoringLevel.L1
        ));
        driverController.rightBumper().toggleOnTrue(superstructure.descoreAlgaeManual(operatorDashboard::getAlgaeDescoringElevatorGoal));

        driverController.povDown().whileTrue(climber.towardsRobot());
        driverController.povUp().whileTrue(climber.awayFromRobot());

        if (mode == Constants.Mode.SIM) {
            driverController.x().onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                            new Pose2d(Units.inchesToMeters(650), Units.inchesToMeters(30), new Rotation2d(Math.random() * 2 * Math.PI))
                    ))
            ));
            driverController.a().onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                            new Pose2d(Units.inchesToMeters(650), Units.inchesToMeters(285), new Rotation2d(Math.random() * 2 * Math.PI))
                    ))
            ));
        }

//        // Lock to 0° when A button is held
//        controller
//                .a()
//                .whileTrue(
//                        DriveCommands.joystickDriveAtAngle(
//                                drive,
//                                () -> -controller.getLeftY(),
//                                () -> -controller.getLeftX(),
//                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
//        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    @Override
    public void periodicBeforeCommands() {
        driverControllerDisconnectedAlert.set(!driverController.isConnected());

        autoNotChosenAlert.set(autoChooser.get() == null);
    }
}
