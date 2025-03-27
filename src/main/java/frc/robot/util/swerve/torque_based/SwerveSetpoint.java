// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/swerve/SwerveSetpoint.java

// MIT License
//
// Copyright (c) 2022 Michael Jansen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot.util.swerve.torque_based;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A setpoint for a swerve drivetrain, containing robot-relative chassis speeds and individual
 * module states
 *
 * @param robotRelativeSpeeds Robot-relative chassis speeds
 * @param moduleStates        Array of individual swerve module states. These will be in FL, FR, BL, BR
 *                            order.
 * @param feedforwards        Feedforwards for each module's drive motor. The arrays in this record will be
 *                            in FL, FR, BL, BR order.
 */
public record SwerveSetpoint(
        ChassisSpeeds robotRelativeSpeeds,
        SwerveModuleState[] moduleStates,
        DriveFeedforwards feedforwards
) {}