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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PIDF;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim extends ModuleIO {
    public static final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
            // Specify Configuration
            DriveTrainSimulationConfig.Default()
                    // Specify gyro type (for realistic gyro drifting and error simulation)
                    .withGyro(COTS.ofPigeon2())
                    // Specify swerve module (for realistic swerve dynamics)
                    .withSwerveModule(COTS.ofMark4(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getNEO(1),
                            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                            2 // L2 Gear ratio
                    ))
                    // Configures the track length and track width (spacing between swerve modules)
                    .withTrackLengthTrackWidth(Meters.of(driveConfig.trackLengthMeters()), Meters.of(driveConfig.trackWidthMeters()))
                    // Configures the bumper size (dimensions of the robot bumper)
                    .withBumperSize(Meters.of(driveConfig.bumperLengthMeters()), Meters.of(driveConfig.bumperWidthMeters()))
                    .withRobotMass(Pounds.of(125)),
            // Specify starting pose
            new Pose2d(3, 3, new Rotation2d())
    );

    static {
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = moduleConfig.driveGains().toPID();
    private PIDController turnController = moduleConfig.turnGains().toPIDWrapRadians();
    private SimpleMotorFeedforward driveFF = moduleConfig.driveGains().toSimpleFF();
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(int index) {
        moduleSimulation = driveSimulation.getModules()[index];

        driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(moduleConfig.driveCurrentLimit()));
        turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(moduleConfig.turnCurrentLimit()));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotor.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
        turnMotor.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnAbsoluteEncoderConnected = true;
        inputs.turnPositionRad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
        inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

        // Update odometry inputs
        inputs.odometryTimestamps = getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositionsRad = Arrays.stream(moduleSimulation.getCachedSteerAbsolutePositions())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }

    @Override
    public void setDrivePIDF(PIDF newGains) {
        System.out.println("Setting drive gains");
        driveFF = newGains.toSimpleFF();
        driveController = newGains.toPID();
    }

    @Override
    public void setTurnPIDF(PIDF newGains) {
        System.out.println("Setting turn gains");
        turnController = newGains.toPID();
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        System.out.println("Setting drive brake mode to " + enable);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        System.out.println("Setting turn brake mode to " + enable);
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = driveFF.calculate(velocityRadPerSec);
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(double positionRad) {
        turnClosedLoop = true;
        turnController.setSetpoint(positionRad);
    }

    protected static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getTimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
