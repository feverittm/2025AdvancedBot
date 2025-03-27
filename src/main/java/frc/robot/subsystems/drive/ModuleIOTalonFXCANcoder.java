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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.HighFrequencySamplingThread;
import frc.robot.util.PIDF;

import java.util.Queue;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.PhoenixUtil.tryUntilOkAsync;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXCANcoder extends ModuleIO {
    private static final SwerveModuleConstants.ClosedLoopOutputType steerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrentAmps;
    private final StatusSignal<Temperature> driveTemperatureCelsius;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrentAmps;
    private final StatusSignal<Temperature> turnTemperatureCelsius;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFXCANcoder(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        driveTalon = new TalonFX(driveCanID, Constants.CANivore.busName);
        turnTalon = new TalonFX(turnCanID, Constants.CANivore.busName);
        cancoder = new CANcoder(cancoderCanID, Constants.CANivore.busName);

        // Configure drive motor
        driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = Slot0Configs.from(moduleConfig.driveGains().toPhoenix());
        driveConfig.Feedback.SensorToMechanismRatio = moduleConfig.driveGearRatio();
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = moduleConfig.driveCurrentLimit();
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = moduleConfig.driveCurrentLimit();
        driveConfig.CurrentLimits.StatorCurrentLimit = moduleConfig.driveCurrentLimit();
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
                moduleConfig.driveInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = Slot0Configs.from(moduleConfig.turnGains().toPhoenix(StaticFeedforwardSignValue.UseClosedLoopSign));
        turnConfig.Feedback.FeedbackRemoteSensorID = cancoderCanID;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.RotorToSensorRatio = moduleConfig.turnGearRatio();
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = moduleConfig.turnCurrentLimit();
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = moduleConfig.turnCurrentLimit();
        turnConfig.CurrentLimits.StatorCurrentLimit = moduleConfig.turnCurrentLimit();
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / moduleConfig.turnGearRatio();
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * moduleConfig.turnGearRatio();
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted =
                moduleConfig.turnInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(-absoluteEncoderOffsetRad);
        cancoderConfig.MagnetSensor.SensorDirection =
                moduleConfig.encoderInverted()
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

        // Create timestamp queue
        timestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue = HighFrequencySamplingThread.get().registerPhoenixSignal(driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrentAmps = driveTalon.getStatorCurrent();
        driveTemperatureCelsius = driveTalon.getDeviceTemp();

        // Create turn status signals
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = HighFrequencySamplingThread.get().registerPhoenixSignal(turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrentAmps = turnTalon.getStatorCurrent();
        turnTemperatureCelsius = driveTalon.getDeviceTemp();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(HighFrequencySamplingThread.frequencyHz, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrentAmps,
                driveTemperatureCelsius,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrentAmps,
                turnTemperatureCelsius
        );
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrentAmps, driveTemperatureCelsius);
        var turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrentAmps, turnTemperatureCelsius);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
        inputs.driveTemperatureCelsius = driveTemperatureCelsius.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnAbsoluteEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPositionRad = Units.rotationsToRadians(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrentAmps.getValueAsDouble();
        inputs.turnTemperatureCelsius = turnTemperatureCelsius.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDrivePIDF(PIDF newGains) {
        System.out.println("Setting drive gains");
        driveConfig.Slot0 = Slot0Configs.from(newGains.toPhoenix());
        tryUntilOkAsync(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setTurnPIDF(PIDF newGains) {
        System.out.println("Setting turn gains");
        turnConfig.Slot0 = Slot0Configs.from(newGains.toPhoenix());
        tryUntilOkAsync(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOkAsync(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOkAsync(5, () -> driveTalon.getConfigurator().apply(turnConfig, 0.25));
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(switch (driveClosedLoopOutput) {
            case Voltage -> voltageRequest.withOutput(output);
            case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnTalon.setControl(switch (steerClosedLoopOutput) {
            case Voltage -> voltageRequest.withOutput(output);
            case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveTalon.setControl(switch (driveClosedLoopOutput) {
            case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
            case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
    }

    @Override
    public void setTurnPosition(double positionRad) {
        double positionRot = Units.radiansToRotations(positionRad);
        turnTalon.setControl(switch (steerClosedLoopOutput) {
            case Voltage -> positionVoltageRequest.withPosition(positionRot);
            case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(positionRot);
        });
    }
}
