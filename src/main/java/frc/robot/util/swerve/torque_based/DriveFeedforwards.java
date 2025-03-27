// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/DriveFeedforwards.java

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

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Collection of different feedforward values for each drive module. If using swerve, these values
 * will all be in FL, FR, BL, BR order. If using a differential drive, these will be in L, R order.
 *
 * <p>NOTE: If using Choreo paths, all feedforwards but the X and Y component arrays will be filled
 * with zeros.
 *
 * @param accelerationsMetersPerSecSquared Linear acceleration at the wheels in meters per second
 * @param linearForcesNewtons              Linear force applied by the motors at the wheels in newtons
 * @param torqueCurrentsAmps               Torque-current of the drive motors in amps
 * @param robotRelativeForcesXNewtons      X components of robot-relative force vectors for the wheels in
 *                                         newtons. The magnitude of these vectors will typically be greater than the linear force
 *                                         feedforwards due to friction forces.
 * @param robotRelativeForcesYNewtons      X components of robot-relative force vectors for the wheels in
 *                                         newtons. The magnitude of these vectors will typically be greater than the linear force
 *                                         feedforwards due to friction forces.
 */
public record DriveFeedforwards(
        double[] accelerationsMetersPerSecSquared,
        double[] linearForcesNewtons,
        double[] torqueCurrentsAmps,
        double[] robotRelativeForcesXNewtons,
        double[] robotRelativeForcesYNewtons)
        implements Interpolatable<DriveFeedforwards> {
    /**
     * Create drive feedforwards consisting of all zeros
     *
     * @param numModules Number of drive modules
     *
     * @return Zero feedforwards
     */
    public static DriveFeedforwards zeros(int numModules) {
        return new DriveFeedforwards(
                new double[numModules],
                new double[numModules],
                new double[numModules],
                new double[numModules],
                new double[numModules]);
    }

    @Override
    public DriveFeedforwards interpolate(DriveFeedforwards endValue, double t) {
        return new DriveFeedforwards(
                interpolateArray(accelerationsMetersPerSecSquared, endValue.accelerationsMetersPerSecSquared, t),
                interpolateArray(linearForcesNewtons, endValue.linearForcesNewtons, t),
                interpolateArray(torqueCurrentsAmps, endValue.torqueCurrentsAmps, t),
                interpolateArray(robotRelativeForcesXNewtons, endValue.robotRelativeForcesXNewtons, t),
                interpolateArray(robotRelativeForcesYNewtons, endValue.robotRelativeForcesYNewtons, t));
    }

    /**
     * Reverse the feedforwards for driving backwards. This should only be used for differential drive
     * robots.
     *
     * @return Reversed feedforwards
     */
    public DriveFeedforwards reverse() {
        if (accelerationsMetersPerSecSquared.length != 2) {
            throw new IllegalStateException(
                    "Feedforwards should only be reversed for differential drive trains");
        }

        return new DriveFeedforwards(
                new double[]{-accelerationsMetersPerSecSquared[1], -accelerationsMetersPerSecSquared[0]},
                new double[]{-linearForcesNewtons[1], -linearForcesNewtons[0]},
                new double[]{-torqueCurrentsAmps[1], -torqueCurrentsAmps[0]},
                new double[]{-robotRelativeForcesXNewtons[1], -robotRelativeForcesXNewtons[0]},
                new double[]{-robotRelativeForcesYNewtons[1], -robotRelativeForcesYNewtons[0]});
    }

    /**
     * Flip the feedforwards for the other side of the field. Only does anything if mirrored symmetry
     * is used
     *
     * @return Flipped feedforwards
     */
    public DriveFeedforwards flip() {
        return new DriveFeedforwards(
                FlippingUtil.flipFeedforwards(accelerationsMetersPerSecSquared),
                FlippingUtil.flipFeedforwards(linearForcesNewtons),
                FlippingUtil.flipFeedforwards(torqueCurrentsAmps),
                FlippingUtil.flipFeedforwardXs(robotRelativeForcesXNewtons),
                FlippingUtil.flipFeedforwardYs(robotRelativeForcesYNewtons));
    }

    private static double[] interpolateArray(double[] a, double[] b, double t) {
        double[] ret = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            ret[i] = MathUtil.interpolate(a[i], b[i], t);
        }
        return ret;
    }
}