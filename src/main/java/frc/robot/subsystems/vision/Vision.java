// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagIO.PoseObservationType;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class Vision extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    private final AprilTagIO[] aprilTagIo = createAprilTagIO();
    private final GamepieceIO[] gamepieceIo = createGamepieceIO();
    private final AprilTagIOInputsAutoLogged[] atInputs;
    private final GamepieceIOInputsAutoLogged[] gpInputs;

    private final Alert[] atDisconnectedAlerts;
    private final Alert[] gpDisconnectedAlerts;

    @Getter
    private Optional<Translation2d> closestGamepiece = Optional.empty();

    private static Vision instance;

    public static Vision get() {
        if (instance == null)
            synchronized (Vision.class) {
                instance = new Vision();
            }

        return instance;
    }

    private Vision() {
        // Initialize inputs
        this.atInputs = new AprilTagIOInputsAutoLogged[aprilTagIo.length];
        this.gpInputs = new GamepieceIOInputsAutoLogged[gamepieceIo.length];
        for (int i = 0; i < atInputs.length; i++) {
            atInputs[i] = new AprilTagIOInputsAutoLogged();
        }
        for (int i = 0; i < gpInputs.length; i++) {
            gpInputs[i] = new GamepieceIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.atDisconnectedAlerts = new Alert[aprilTagIo.length];
        for (int i = 0; i < atInputs.length; i++) {
            atDisconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }

        this.gpDisconnectedAlerts = new Alert[gamepieceIo.length];
        for (int i = 0; i < gpInputs.length; i++) {
            gpDisconnectedAlerts[i] =
                    new Alert(
                            "Gampiece camera " + i + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        for (int i = 0; i < aprilTagIo.length; i++) {
            aprilTagIo[i].updateInputs(atInputs[i]);
            Logger.processInputs("Inputs/Vision/Camera" + i, atInputs[i]);
        }

        for (int i = 0; i < gamepieceIo.length; i++) {
            gamepieceIo[i].updateInputs(gpInputs[i]);
            Logger.processInputs("Inputs/Vision/Gamepiece" + i, gpInputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < aprilTagIo.length; cameraIndex++) {
            // Update disconnected alert
            atDisconnectedAlerts[cameraIndex].set(!atInputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : atInputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : atInputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity if only one tap
                                || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate
                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                RobotState.get().addVisionMeasurement(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()])
            );
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()])
            );
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()])
            );
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()])
            );
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        var robotTranslation = robotState.getTranslation();
        var allGamepieces = new ArrayList<Translation2d>();
        for (int cameraIndex = 0; cameraIndex < gamepieceIo.length; cameraIndex++) {
            // Update disconnected alert
            gpDisconnectedAlerts[cameraIndex].set(!gpInputs[cameraIndex].connected);

            // TODO: replace with algorithm getting closest if there is more than one targets
            var present = gpInputs[cameraIndex].latestGamepieceTargetObservation.isPresent();

            Logger.recordOutput("Vision/Gamepiece" + cameraIndex + "/TargetPresent", present);

            if (present) {
                var closestTarget = gpInputs[cameraIndex].latestGamepieceTargetObservation.targetPos();
                var closestTargetAbsolute = robotTranslation.plus(closestTarget);

                Logger.recordOutput(
                        "Vision/Gamepiece" + cameraIndex + "/ClosestPose",
                        new Pose3d(closestTargetAbsolute.getX(), closestTargetAbsolute.getY(), 0, new Rotation3d()));

                allGamepieces.add(closestTargetAbsolute);
            }
        }
        closestGamepiece = Optional.empty();
        for (var gamepiece : allGamepieces) {
            if (closestGamepiece.isEmpty() || robotTranslation.getDistance(gamepiece) < robotTranslation.getDistance(closestGamepiece.get())) {
                closestGamepiece = Optional.of(gamepiece);
            }
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        Logger.recordOutput("Vision/Summary/ClosestGamepiece/Present", closestGamepiece.isPresent());
        closestGamepiece.ifPresent(translation2d -> {
            var asPose3d = new Pose3d(translation2d.getX(), translation2d.getY(), 0, new Rotation3d());
            Logger.recordOutput("Vision/Summary/ClosestGamepiece/Pose", asPose3d);
        });

        // Log camera poses for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "Vision/CameraPoses",
                robotPose.transformBy(reefCamRobotToCamera),
                robotPose.transformBy(stationCamRobotToCamera)
        );
    }
}
