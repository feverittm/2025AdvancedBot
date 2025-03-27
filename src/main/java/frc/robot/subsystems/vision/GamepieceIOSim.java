package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

public class GamepieceIOSim extends GamepieceIO {
    private final RobotState robotState = RobotState.get();

    public GamepieceIOSim() {
    }

    @Override
    public void updateInputs(GamepieceIO.GamepieceIOInputs inputs) {
        inputs.connected = true;
        inputs.latestGamepieceTargetObservation = new GamepieceTargetObservation(
                new Rotation2d(0),
                new Rotation2d(0),
                new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13)).minus(robotState.getTranslation()),
                true
        );
    }
}
