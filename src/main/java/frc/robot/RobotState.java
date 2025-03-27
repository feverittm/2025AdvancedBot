package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.Supplier;

public class RobotState {
    @Getter
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            },
            new Pose2d());

    private static RobotState instance;

    public static RobotState get() {
        if (instance == null)
            synchronized (RobotState.class) {
                instance = new RobotState();
            }

        return instance;
    }

    private RobotState() {
    }

    public void applyOdometryUpdate(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions
    ) {
        poseEstimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @AutoLogOutput(key = "RobotState/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        final var drive = Drive.get();
        poseEstimator.resetPosition(drive.getRawGyroRotation(), drive.getMeasuredModulePositions(), pose);
    }

    public Command setPose(Supplier<Pose2d> pose) {
        return Commands
                .runOnce(() -> setPose(pose.get()))
                .ignoringDisable(true);
    }

    public Command resetRotation() {
        return setPose(() -> new Pose2d(getTranslation(), new Rotation2d()));
    }
}
