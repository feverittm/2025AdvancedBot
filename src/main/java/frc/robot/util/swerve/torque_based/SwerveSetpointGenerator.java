// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/swerve/SwerveSetpointGenerator.java

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static frc.robot.Util.epsilonEquals;
import static frc.robot.Util.greaterThanEpsilon;

/**
 * Swerve setpoint generator based on a version created by FRC team 254.
 *
 * <p>Takes a prior setpoint, a desired setpoint, and outputs a new setpoint that respects all the
 * kinematic constraints on module rotation and wheel velocity/torque, as well as preventing any
 * forces acting on a module's wheel from exceeding the force of friction.
 */
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;
    private final SimpleMatrix forceKinematics;
    private final Translation2d[] moduleLocations;
    /** Distance from center of robot to each module in meters */
    private final double[] moduleDistances;

    private final double maxTurnVelocityRadPerSec;
    /** The max RPM that the drive motor can reach while actually driving the robot at full output. */
    private final double maxDriveVelocityMetersPerSec;
    private final double wheelRadiusMeters;
    /** The force of static friction between the robot's drive wheels and the carpet */
    private final double wheelFrictionNewtons;
    private final double massKG;
    /** The moment of inertia of the robot, in KG*M^2 */
    public final double MOIKGMetersSquared;

    /** The DCMotor representing the drive gearbox, including gear reduction */
    private final DCMotor driveMotor;
    /** The current limit of the drive motor, in Amps */
    private final double driveCurrentLimit;
    /** The amount of motor torque lost while driving. Calculated by getting the torque of the motor at the motor's max speed under load. */
    private final double driveTorqueLoss;
    /** The maximum torque a drive module can apply without slipping the wheels */
    public final double driveMaxTorqueWithoutSlip;

    private final double brownoutVoltage;

    public SwerveSetpointGenerator(
            SwerveDriveKinematics kinematics,
            double maxTurnVelocityRadPerSec,
            double maxDriveVelocityMetersPerSec,
            double wheelRadiusMeters,
            double wheelCOF,
            double massKG,
            double MOIKGMetersSquared,
            DCMotor driveMotor,
            double driveCurrentLimit
    ) {
        this.kinematics = kinematics;
        moduleLocations = kinematics.getModules();
        moduleDistances = Arrays.stream(moduleLocations).mapToDouble(Translation2d::getNorm).toArray();
        // https://github.com/mjansen4857/pathplanner/blob/10271416a7d0e6a6296cf0a6be5867af5df67c43/pathplannerlib/src/main/java/com/pathplanner/lib/config/RobotConfig.java#L101
        forceKinematics = new SimpleMatrix(moduleLocations.length * 2, 3);
        for (int i = 0; i < moduleLocations.length; i++) {
            Translation2d modPosReciprocal = new Translation2d(
                    1.0 / moduleLocations[i].getNorm(),
                    moduleLocations[i].getAngle()
            );
            forceKinematics.setRow(i * 2, 0, /* Start Data */ 1, 0, -modPosReciprocal.getY());
            forceKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, modPosReciprocal.getX());
        }

        this.maxTurnVelocityRadPerSec = maxTurnVelocityRadPerSec;
        this.maxDriveVelocityMetersPerSec = maxDriveVelocityMetersPerSec;
        this.wheelRadiusMeters = wheelRadiusMeters;
        this.massKG = massKG;
        this.MOIKGMetersSquared = MOIKGMetersSquared;
        this.driveMotor = driveMotor;
        this.driveCurrentLimit = driveCurrentLimit;

        // https://github.com/mjansen4857/pathplanner/blob/10271416a7d0e6a6296cf0a6be5867af5df67c43/pathplannerlib/src/main/java/com/pathplanner/lib/config/RobotConfig.java#L98
        this.wheelFrictionNewtons = wheelCOF * ((massKG / moduleLocations.length) * 9.8);
        // https://github.com/mjansen4857/pathplanner/blob/10271416a7d0e6a6296cf0a6be5867af5df67c43/pathplannerlib/src/main/java/com/pathplanner/lib/config/RobotConfig.java#L99
        this.driveMaxTorqueWithoutSlip = wheelFrictionNewtons * wheelRadiusMeters;
        // https://github.com/mjansen4857/pathplanner/blob/10271416a7d0e6a6296cf0a6be5867af5df67c43/pathplannerlib/src/main/java/com/pathplanner/lib/config/ModuleConfig.java#L59
        double maxDriveVelocityRadPerSec = maxDriveVelocityMetersPerSec / this.wheelRadiusMeters;
        double maxSpeedCurrentDraw = this.driveMotor.getCurrent(maxDriveVelocityRadPerSec, 12.0);
        this.driveTorqueLoss = Math.max(
                this.driveMotor.getTorque(Math.min(maxSpeedCurrentDraw, this.driveCurrentLimit)),
                0.0
        );

        this.brownoutVoltage = RobotController.getBrownoutVoltage();
    }

    /**
     * Generate a new setpoint with explicit battery voltage. Note: Do not discretize ChassisSpeeds
     * passed into or returned from this method. This method will discretize the speeds for you.
     *
     * @param prevSetpoint              The previous setpoint motion. Normally, you'd pass in the previous
     *                                  iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
     *                                  a path following algorithm.
     * @param constraints               The arbitrary constraints to respect along with the robot's max
     *                                  capabilities. If this is null, the generator will only limit setpoints by the robot's max
     *                                  capabilities.
     * @param dt                        The loop time.
     * @param inputVoltage              The input voltage of the drive motor controllers, in volts. This can also
     *                                  be a static nominal voltage if you do not want the setpoint generator to react to changes
     *                                  in input voltage. If the given voltage is NaN, it will be assumed to be 12v. The input
     *                                  voltage will be clamped to a minimum of the robot controller's brownout voltage.
     *
     * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
     * desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            final SwerveConstraints constraints,
            double dt,
            double inputVoltage) {
        if (Double.isNaN(inputVoltage)) {
            inputVoltage = 12.0;
        } else {
            inputVoltage = Math.max(inputVoltage, brownoutVoltage);
        }
        double maxSpeed = maxDriveVelocityMetersPerSec * Math.min(1, inputVoltage / 12);

        // Limit the max velocities in desired state based on constraints
        if (constraints != null) {
            Translation2d vel = new Translation2d(
                    desiredStateRobotRelative.vxMetersPerSecond,
                    desiredStateRobotRelative.vyMetersPerSecond
            );
            double linearVel = vel.getNorm();
            if (linearVel > constraints.maxVelocityMetersPerSec()) {
                vel = vel.times(constraints.maxVelocityMetersPerSec() / linearVel);
            }
            desiredStateRobotRelative = new ChassisSpeeds(
                    vel.getX(),
                    vel.getY(),
                    MathUtil.clamp(
                            desiredStateRobotRelative.omegaRadiansPerSecond,
                            -constraints.maxAngularVelocityRadPerSec(),
                            constraints.maxAngularVelocityRadPerSec()
                    )
            );
        }

        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredStateRobotRelative);
        // Make sure desiredState respects velocity limits.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeed);
        desiredStateRobotRelative = kinematics.toChassisSpeeds(desiredModuleStates);

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
        // just use the previous angle.
        boolean need_to_steer = true;
        if (epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
            need_to_steer = false;
            for (int m = 0; m < moduleLocations.length; m++) {
                desiredModuleStates[m].angle = prevSetpoint.moduleStates()[m].angle;
                desiredModuleStates[m].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = new double[moduleLocations.length];
        double[] prev_vy = new double[moduleLocations.length];
        Rotation2d[] prev_heading = new Rotation2d[moduleLocations.length];
        double[] desired_vx = new double[moduleLocations.length];
        double[] desired_vy = new double[moduleLocations.length];
        Rotation2d[] desired_heading = new Rotation2d[moduleLocations.length];
        boolean all_modules_should_flip = true;
        for (int m = 0; m < moduleLocations.length; m++) {
            prev_vx[m] =
                    prevSetpoint.moduleStates()[m].angle.getCos()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prev_vy[m] =
                    prevSetpoint.moduleStates()[m].angle.getSin()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prev_heading[m] = prevSetpoint.moduleStates()[m].angle;
            if (prevSetpoint.moduleStates()[m].speedMetersPerSecond < 0.0) {
                prev_heading[m] = prev_heading[m].rotateBy(Rotation2d.k180deg);
            }
            desired_vx[m] =
                    desiredModuleStates[m].angle.getCos() * desiredModuleStates[m].speedMetersPerSecond;
            desired_vy[m] =
                    desiredModuleStates[m].angle.getSin() * desiredModuleStates[m].speedMetersPerSecond;
            desired_heading[m] = desiredModuleStates[m].angle;
            if (desiredModuleStates[m].speedMetersPerSecond < 0.0) {
                desired_heading[m] = desired_heading[m].rotateBy(Rotation2d.k180deg);
            }
            if (all_modules_should_flip) {
                double required_rotation_rad =
                        Math.abs(prev_heading[m].unaryMinus().rotateBy(desired_heading[m]).getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip
                && !epsilonEquals(prevSetpoint.robotRelativeSpeeds(), new ChassisSpeeds())
                && !epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
            // of the desired angle, and accelerate again.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds(), constraints, dt, inputVoltage);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to
        // the goal state; then find the amount we can move from start towards goal in this cycle such
        // that no kinematic limit is exceeded.
        double dx = desiredStateRobotRelative.vxMetersPerSecond
                - prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
        double dy = desiredStateRobotRelative.vyMetersPerSecond
                - prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
        double dtheta = desiredStateRobotRelative.omegaRadiansPerSecond
                - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
        // desiredState.
        double min_s = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle
        // to command (since inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(moduleLocations.length);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
        // the current angle, and then backing out the maximum interpolant between start and goal
        // states. We remember the minimum across all modules, since that is the active constraint.
        for (int m = 0; m < moduleLocations.length; m++) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[m].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());

            double max_theta_step = dt * maxTurnVelocityRadPerSec;

            if (epsilonEquals(prevSetpoint.moduleStates()[m].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering
                // angle, so limit based purely on rotation in place.
                if (epsilonEquals(desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(m, Optional.of(prevSetpoint.moduleStates()[m].angle));
                    continue;
                }

                var necessaryRotation = prevSetpoint
                        .moduleStates()[m]
                        .angle
                        .unaryMinus()
                        .rotateBy(desiredModuleStates[m].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
                }

                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(m, Optional.of(desiredModuleStates[m].angle));
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(
                            m,
                            Optional.of(
                                    prevSetpoint.moduleStates()[m].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians()) * max_theta_step
                                            )
                                    )
                            )
                    );
                    min_s = 0.0;
                }
                continue;
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            // Enforce centripetal force limits to prevent sliding.
            // We do this by changing max_theta_step to the maximum change in heading over dt
            // that would create a large enough radius to keep the centripetal force under the
            // friction force.
            double maxHeadingChange =
                    (dt * wheelFrictionNewtons)
                            / ((massKG / moduleLocations.length) * Math.abs(prevSetpoint.moduleStates()[m].speedMetersPerSecond));
            max_theta_step = Math.min(max_theta_step, maxHeadingChange);

            double s = findSteeringMaxS(
                    prev_vx[m],
                    prev_vy[m],
                    prev_heading[m].getRadians(),
                    desired_vx[m],
                    desired_vy[m],
                    desired_heading[m].getRadians(),
                    max_theta_step
            );
            min_s = Math.min(min_s, s);
        }

        // Enforce drive wheel torque limits
        Translation2d chassisForceVec = new Translation2d();
        double chassisTorque = 0.0;
        for (int m = 0; m < moduleLocations.length; m++) {
            double lastVelRadPerSec = prevSetpoint.moduleStates()[m].speedMetersPerSecond / wheelRadiusMeters;
            // Use the current battery voltage since we won't be able to supply 12v if the
            // battery is sagging down to 11v, which will affect the max torque output
            double currentDraw = driveMotor.getCurrent(Math.abs(lastVelRadPerSec), inputVoltage);
            double reverseCurrentDraw = Math.abs(driveMotor.getCurrent(Math.abs(lastVelRadPerSec), -inputVoltage));
            currentDraw = Math.min(currentDraw, driveCurrentLimit);
            currentDraw = Math.max(currentDraw, 0);
            reverseCurrentDraw = Math.min(reverseCurrentDraw, driveCurrentLimit);
            reverseCurrentDraw = Math.max(reverseCurrentDraw, 0);
            double forwardModuleTorque = driveMotor.getTorque(currentDraw);
            double reverseModuleTorque = driveMotor.getTorque(reverseCurrentDraw);

            double prevSpeed = prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            desiredModuleStates[m].optimize(prevSetpoint.moduleStates()[m].angle);
            double desiredSpeed = desiredModuleStates[m].speedMetersPerSecond;

            int forceSign;
            Rotation2d forceAngle = prevSetpoint.moduleStates()[m].angle;
            double moduleTorque;
            if (epsilonEquals(prevSpeed, 0.0)
                    || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
                    || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
                moduleTorque = forwardModuleTorque;
                // Torque loss will be fighting motor
                moduleTorque -= driveTorqueLoss;
                forceSign = 1; // Force will be applied in direction of module
                if (prevSpeed < 0) {
                    forceAngle = forceAngle.plus(Rotation2d.k180deg);
                }
            } else {
                moduleTorque = reverseModuleTorque;
                // Torque loss will be helping the motor
                moduleTorque += driveTorqueLoss;
                forceSign = -1; // Force will be applied in opposite direction of module
                if (prevSpeed > 0) {
                    forceAngle = forceAngle.plus(Rotation2d.k180deg);
                }
            }

            // Limit torque to prevent wheel slip
            moduleTorque = Math.min(moduleTorque, driveMaxTorqueWithoutSlip);

            double forceAtCarpet = moduleTorque / wheelRadiusMeters;
            Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);

            // Add the module force vector to the chassis force vector
            chassisForceVec = chassisForceVec.plus(moduleForceVec);

            // Calculate the torque this module will apply to the chassis
            if (!epsilonEquals(0, moduleForceVec.getNorm())) {
                Rotation2d angleToModule = moduleLocations[m].getAngle();
                Rotation2d theta = moduleForceVec.getAngle().minus(angleToModule);
                chassisTorque += forceAtCarpet * moduleDistances[m] * theta.getSin();
            }
        }

        Translation2d chassisAccelVec = chassisForceVec.div(massKG);
        double chassisAngularAccel = chassisTorque / MOIKGMetersSquared;

        if (constraints != null) {
            double linearAccel = chassisAccelVec.getNorm();
            if (linearAccel > constraints.maxAccelerationMetersPerSecSquared()) {
                chassisAccelVec = chassisAccelVec.times(constraints.maxAccelerationMetersPerSecSquared() / linearAccel);
            }
            chassisAngularAccel =
                    MathUtil.clamp(
                            chassisAngularAccel,
                            -constraints.maxAngularAccelerationRadPerSecSquared(),
                            constraints.maxAngularAccelerationRadPerSecSquared());
        }

        // Use kinematics to convert chassis accelerations to module accelerations
        ChassisSpeeds chassisAccel = new ChassisSpeeds(chassisAccelVec.getX(), chassisAccelVec.getY(), chassisAngularAccel);
        var accelStates = kinematics.toSwerveModuleStates(chassisAccel);

        for (int m = 0; m < moduleLocations.length; m++) {
            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }

            double maxVelStep = Math.abs(accelStates[m].speedMetersPerSecond * dt);

            double vx_min_s = min_s == 1.0
                    ? desired_vx[m]
                    : (desired_vx[m] - prev_vx[m]) * min_s + prev_vx[m];
            double vy_min_s = min_s == 1.0
                    ? desired_vy[m]
                    : (desired_vy[m] - prev_vy[m]) * min_s + prev_vy[m];
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
            // already know we can't go faster than that.
            double s = findDriveMaxS(prev_vx[m], prev_vy[m], vx_min_s, vy_min_s, maxVelStep);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond + min_s * dx,
                prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond + min_s * dy,
                prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond + min_s * dtheta
        );
        retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);

        double prevVelX = prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
        double prevVelY = prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
        double chassisAccelX = (retSpeeds.vxMetersPerSecond - prevVelX) / dt;
        double chassisAccelY = (retSpeeds.vyMetersPerSecond - prevVelY) / dt;
        double chassisForceX = chassisAccelX * massKG;
        double chassisForceY = chassisAccelY * massKG;

        double angularAccel = (retSpeeds.omegaRadiansPerSecond - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond) / dt;
        double angTorque = angularAccel * MOIKGMetersSquared;
        ChassisSpeeds chassisForces = new ChassisSpeeds(chassisForceX, chassisForceY, angTorque);

        Translation2d[] wheelForces = chassisForcesToWheelForceVectors(chassisForces);

        var retStates = kinematics.toSwerveModuleStates(retSpeeds);
        double[] accelFF = new double[moduleLocations.length];
        double[] linearForceFF = new double[moduleLocations.length];
        double[] torqueCurrentFF = new double[moduleLocations.length];
        double[] forceXFF = new double[moduleLocations.length];
        double[] forceYFF = new double[moduleLocations.length];
        for (int m = 0; m < moduleLocations.length; m++) {
            double wheelForceDist = wheelForces[m].getNorm();
            double appliedForce =
                    greaterThanEpsilon(wheelForceDist)
                            ? wheelForceDist * wheelForces[m].getAngle().minus(retStates[m].angle).getCos()
                            : 0.0;
            double wheelTorque = appliedForce * wheelRadiusMeters;
            double torqueCurrent = driveMotor.getCurrent(wheelTorque);

            final var maybeOverride = overrideSteering.get(m);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(retStates[m].angle.unaryMinus().rotateBy(override))) {
                    retStates[m].speedMetersPerSecond *= -1.0;
                    appliedForce *= -1.0;
                    torqueCurrent *= -1.0;
                }
                retStates[m].angle = override;
            }
            final var deltaRotation =
                    prevSetpoint.moduleStates()[m].angle.unaryMinus().rotateBy(retStates[m].angle);
            if (flipHeading(deltaRotation)) {
                retStates[m].angle = retStates[m].angle.rotateBy(Rotation2d.k180deg);
                retStates[m].speedMetersPerSecond *= -1.0;
                appliedForce *= -1.0;
                torqueCurrent *= -1.0;
            }

            accelFF[m] = (retStates[m].speedMetersPerSecond - prevSetpoint.moduleStates()[m].speedMetersPerSecond) / dt;
            linearForceFF[m] = appliedForce;
            torqueCurrentFF[m] = torqueCurrent;
            forceXFF[m] = wheelForces[m].getX();
            forceYFF[m] = wheelForces[m].getY();
        }

        return new SwerveSetpoint(
                retSpeeds,
                retStates,
                new DriveFeedforwards(accelFF, linearForceFF, torqueCurrentFF, forceXFF, forceYFF)
        );
    }

    /**
     * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
     * this method. This method will discretize the speeds for you.
     *
     * <p>Note: This method will automatically use the current robot controller input voltage.
     *
     * @param prevSetpoint              The previous setpoint motion. Normally, you'd pass in the previous
     *                                  iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
     *                                  a path following algorithm.
     * @param constraints               The arbitrary constraints to respect along with the robot's max
     *                                  capabilities. If this is null, the generator will only limit setpoints by the robot's max
     *                                  capabilities.
     * @param dt                        The loop time.
     *
     * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
     * desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            final SwerveConstraints constraints,
            double dt
    ) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                constraints,
                dt,
                RobotController.getInputVoltage()
        );
    }

    /**
     * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
     * this method. This method will discretize the speeds for you.
     *
     * <p>Note: This method will automatically use the current robot controller input voltage.
     *
     * @param prevSetpoint              The previous setpoint motion. Normally, you'd pass in the previous
     *                                  iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
     *                                  a path following algorithm.
     * @param dt                        The loop time.
     *
     * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
     * desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredStateRobotRelative, double dt) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                null,
                dt,
                RobotController.getInputVoltage()
        );
    }

    /**
     * Generate a new setpoint with explicit battery voltage. Note: Do not discretize ChassisSpeeds
     * passed into or returned from this method. This method will discretize the speeds for you.
     *
     * @param prevSetpoint              The previous setpoint motion. Normally, you'd pass in the previous
     *                                  iteration setpoint instead of the actual measured/estimated kinematic state.
     * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
     *                                  a path following algorithm.
     * @param dt                        The loop time.
     * @param inputVoltage              The input voltage of the drive motor controllers, in volts. This can also
     *                                  be a static nominal voltage if you do not want the setpoint generator to react to changes
     *                                  in input voltage. If the given voltage is NaN, it will be assumed to be 12v. The input
     *                                  voltage will be clamped to a minimum of the robot controller's brownout voltage.
     *
     * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
     * desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            double dt,
            double inputVoltage
    ) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                null,
                dt,
                inputVoltage
        );
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
     * direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e.
     *                   prev.inverse().rotateBy(goal)).
     *
     * @return True if the shortest path to achieve this rotation involves flipping the drive
     * direction.
     */
    private static boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    private static double findSteeringMaxS(
            double x_0,
            double y_0,
            double theta_0,
            double x_1,
            double y_1,
            double theta_1,
            double max_deviation) {
        theta_1 = unwrapAngle(theta_0, theta_1);
        double diff = theta_1 - theta_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }

        double target = theta_0 + Math.copySign(max_deviation, diff);

        // Rotate the velocity vectors such that the target angle becomes the +X
        // axis. We only need find the Y components, h_0 and h_1, since they are
        // proportional to the distances from the two points to the solution
        // point (x_0 + (x_1 - x_0)s, y_0 + (y_1 - y_0)s).
        double sin = Math.sin(-target);
        double cos = Math.cos(-target);
        double h_0 = sin * x_0 + cos * y_0;
        double h_1 = sin * x_1 + cos * y_1;

        // Undo linear interpolation from h_0 to h_1:
        // 0 = h_0 + (h_1 - h_0) * s
        // -h_0 = (h_1 - h_0) * s
        // -h_0 / (h_1 - h_0) = s
        // h_0 / (h_0 - h_1) = s
        // Guaranteed to not divide by zero, since if h_0 was equal to h_1, theta_0
        // would be equal to theta_1, which is caught by the difference check.
        return h_0 / (h_0 - h_1);
    }

    private static boolean isValidS(double s) {
        return Double.isFinite(s) && s >= 0 && s <= 1;
    }

    private static double findDriveMaxS(
            double x_0, double y_0, double x_1, double y_1, double max_vel_step) {
        // Derivation:
        // Want to find point P(s) between (x_0, y_0) and (x_1, y_1) where the
        // length of P(s) is the target T. P(s) is linearly interpolated between the
        // points, so P(s) = (x_0 + (x_1 - x_0) * s, y_0 + (y_1 - y_0) * s).
        // Then,
        //     T = sqrt(P(s).x^2 + P(s).y^2)
        //   T^2 = (x_0 + (x_1 - x_0) * s)^2 + (y_0 + (y_1 - y_0) * s)^2
        //   T^2 = x_0^2 + 2x_0(x_1-x_0)s + (x_1-x_0)^2*s^2
        //       + y_0^2 + 2y_0(y_1-y_0)s + (y_1-y_0)^2*s^2
        //   T^2 = x_0^2 + 2x_0x_1s - 2x_0^2*s + x_1^2*s^2 - 2x_0x_1s^2 + x_0^2*s^2
        //       + y_0^2 + 2y_0y_1s - 2y_0^2*s + y_1^2*s^2 - 2y_0y_1s^2 + y_0^2*s^2
        //     0 = (x_0^2 + y_0^2 + x_1^2 + y_1^2 - 2x_0x_1 - 2y_0y_1)s^2
        //       + (2x_0x_1 + 2y_0y_1 - 2x_0^2 - 2y_0^2)s
        //       + (x_0^2 + y_0^2 - T^2).
        //
        // To simplify, we can factor out some common parts:
        // Let l_0 = x_0^2 + y_0^2, l_1 = x_1^2 + y_1^2, and
        // p = x_0 * x_1 + y_0 * y_1.
        // Then we have
        //   0 = (l_0 + l_1 - 2p)s^2 + 2(p - l_0)s + (l_0 - T^2),
        // with which we can solve for s using the quadratic formula.

        double l_0 = x_0 * x_0 + y_0 * y_0;
        double l_1 = x_1 * x_1 + y_1 * y_1;
        double sqrt_l_0 = Math.sqrt(l_0);
        double diff = Math.sqrt(l_1) - sqrt_l_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }

        double target = sqrt_l_0 + Math.copySign(max_vel_step, diff);
        double p = x_0 * x_1 + y_0 * y_1;

        // Quadratic of s
        double a = l_0 + l_1 - 2 * p;
        double b = 2 * (p - l_0);
        double c = l_0 - target * target;
        double root = Math.sqrt(b * b - 4 * a * c);

        // Check if either of the solutions are valid
        // Won't divide by zero because it is only possible for a to be zero if the
        // target velocity is exactly the same or the reverse of the current
        // velocity, which would be caught by the difference check.
        double s_1 = (-b + root) / (2 * a);
        if (isValidS(s_1)) {
            return s_1;
        }
        double s_2 = (-b - root) / (2 * a);
        if (isValidS(s_2)) {
            return s_2;
        }

        // Since we passed the initial max_vel_step check, a solution should exist,
        // but if no solution was found anyway, just don't limit movement
        return 1.0;
    }

    // https://github.com/mjansen4857/pathplanner/blob/10271416a7d0e6a6296cf0a6be5867af5df67c43/pathplannerlib/src/main/java/com/pathplanner/lib/config/RobotConfig.java#L227

    /**
     * Convert chassis forces (passed as ChassisSpeeds) to individual wheel force vectors
     *
     * @param chassisForces The linear X/Y force and torque acting on the whole robot
     *
     * @return Array of individual wheel force vectors
     */
    public Translation2d[] chassisForcesToWheelForceVectors(ChassisSpeeds chassisForces) {
        var chassisForceVector = new SimpleMatrix(3, 1);
        chassisForceVector.setColumn(
                0,
                0,
                chassisForces.vxMetersPerSecond,
                chassisForces.vyMetersPerSecond,
                chassisForces.omegaRadiansPerSecond);

        // Divide the chassis force vector by numModules since force is additive. All module forces will
        // add up to the chassis force
        var moduleForceMatrix = forceKinematics.mult(chassisForceVector.divide(moduleLocations.length));

        Translation2d[] forceVectors = new Translation2d[moduleLocations.length];
        for (int m = 0; m < moduleLocations.length; m++) {
            double x = moduleForceMatrix.get(m * 2, 0);
            double y = moduleForceMatrix.get(m * 2 + 1, 0);

            forceVectors[m] = new Translation2d(x, y);
        }

        return forceVectors;
    }
}