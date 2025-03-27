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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.25;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.1; // Meters
    public static double angularStdDevBaseline = Units.degreesToRadians(15); // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[]{
                    1.0, // StationCam
                    0.5 // ReefCam
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    public static GamepieceIO[] createGamepieceIO() {
        return switch (Constants.identity) {
        /*
        case COMPBOT -> Constants.isReplay
                ? new GamepieceIO[]{new GamepieceIO()}
                : new GamepieceIO[]{new GamepieceIOLimelight("limelight", new Transform3d())};
         */
            case COMPBOT -> new GamepieceIO[]{};
            case ALPHABOT -> Constants.isReplay
                    ? new GamepieceIO[]{new GamepieceIO()}
                    : new GamepieceIO[]{
                    new GamepieceIOLimelight(
                            "limelight",
                            // 2 inches back, 2 inches right, 37 inches up, 40 degrees down from horizontal
                            new Transform3d(Units.inchesToMeters(-2), Units.inchesToMeters(-2), Units.inchesToMeters(37),
                                    new Rotation3d(0, Units.degreesToRadians(-40), 0)
                            )
                    )
            };
            case SIMBOT -> Constants.isReplay
                    ? new GamepieceIO[]{new GamepieceIO()}
                    : new GamepieceIO[]{new GamepieceIOSim()};
        };
    }

    public static final Transform3d stationCamRobotToCamera = new Transform3d(
            Units.inchesToMeters(-6.7), Units.inchesToMeters(-9.4), Units.inchesToMeters(27.3),
            // Rotation order matters
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-60)))
    );
    public static final Transform3d reefCamRobotToCamera = new Transform3d(
            Units.inchesToMeters(-8.5), Units.inchesToMeters(8.8), Units.inchesToMeters(25.5),
//             Rotation order matters
            new Rotation3d(0.0, Units.degreesToRadians(35), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-170)))
    );

    public static AprilTagIO[] createAprilTagIO() {
        return switch (Constants.identity) {
            case COMPBOT -> Constants.isReplay
                    ? new AprilTagIO[]{new AprilTagIO(), new AprilTagIO()}
                    : new AprilTagIO[]{
                    new AprilTagIOPhotonVision("StationCam", stationCamRobotToCamera),
                    new AprilTagIOPhotonVision("ReefCam", reefCamRobotToCamera)
            };
//        case ALPHABOT -> Constants.isReplay
//                ? new VisionIO[]{new VisionIO()}
//                : new VisionIO[]{
//                new VisionIOPhotonVision(
//                        "camera_0",
//                        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0))
//                )
//        };
            case ALPHABOT -> new AprilTagIO[]{};
            case SIMBOT -> Constants.isReplay
                    ? new AprilTagIO[]{new AprilTagIO(), new AprilTagIO()}
                    : new AprilTagIO[]{
                    new AprilTagIOPhotonVisionSim(
                            "StationCam",
                            stationCamRobotToCamera
                    ),
                    new AprilTagIOPhotonVisionSim(
                            "ReefCam",
                            reefCamRobotToCamera
                    )
            };
        };
    }
}
