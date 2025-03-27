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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public class GamepieceIO {
    @AutoLog
    public static class GamepieceIOInputs {
        public boolean connected = false;
        // x angle from camera, y angle from camera, translation from robot to gamepiece, whether there is a target
        public GamepieceTargetObservation latestGamepieceTargetObservation = new GamepieceTargetObservation(new Rotation2d(), new Rotation2d(), new Translation2d(), false);
    }

    /**
     * Represents the angle to a simple target, not used for pose estimation.
     */
    public record GamepieceTargetObservation(Rotation2d tx, Rotation2d ty, Translation2d targetPos,
                                             boolean isPresent) {
    }

    public void updateInputs(GamepieceIOInputs inputs) {
    }
}
