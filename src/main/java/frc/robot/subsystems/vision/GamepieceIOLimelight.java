package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotController;

public class GamepieceIOLimelight extends GamepieceIO {
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final StringSubscriber tclassSubscriber;
    private final IntegerSubscriber tvSubscriber;
    private final Transform3d robotToCam;

    public GamepieceIOLimelight(String name, Transform3d robotToCam) {
        var table = NetworkTableInstance.getDefault().getTable(name);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        tclassSubscriber = table.getStringTopic("tclass").subscribe("");
        tvSubscriber = table.getIntegerTopic("tv").subscribe(0);
        this.robotToCam = robotToCam;
    }


    @Override
    public void updateInputs(GamepieceIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        double tx = txSubscriber.get();
        double ty = tySubscriber.get();
        double dist = robotToCam.getZ() / Math.tan(Units.degreesToRadians(-ty) - robotToCam.getRotation().getY());

        // Update target observation
        inputs.latestGamepieceTargetObservation =
                new GamepieceTargetObservation(
                        Rotation2d.fromDegrees(tx),
                        Rotation2d.fromDegrees(ty),
                        new Translation2d(dist, new Rotation2d(robotToCam.getRotation().getZ() - Units.degreesToRadians(tx))).plus(new Translation2d(robotToCam.getX(), robotToCam.getY())),
                        tvSubscriber.get() == 1
                );

        NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight
    }
}
