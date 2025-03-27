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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.util.HighFrequencySamplingThread;
import frc.robot.util.PIDF;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max turn motor controller,
 * and CANcoder.
 */
public class ModuleIOSparkMaxCANcoder extends ModuleIO {
    // Hardware objects
    private final SparkMax driveSpark;
    private final SparkMax turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;
    private final CANcoder cancoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final StatusSignal<Angle> turnAbsolutePosition;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private SimpleMotorFeedforward driveFF = moduleConfig.driveGains().toSimpleFF();

    public ModuleIOSparkMaxCANcoder(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        driveSpark = new SparkMax(driveCanID, MotorType.kBrushless);
        turnSpark = new SparkMax(turnCanID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderCanID, Constants.CANivore.busName);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        driveConfig = new SparkMaxConfig();
        driveConfig
                .inverted(moduleConfig.driveInverted())
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(moduleConfig.driveCurrentLimit())
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(2 * Math.PI / moduleConfig.driveGearRatio()) // Rotor Rotations -> Wheel Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / moduleConfig.driveGearRatio()) // Rotor RPM -> Wheel Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        moduleConfig.driveGains().applySparkPID(driveConfig.closedLoop, ClosedLoopSlot.kSlot0);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / HighFrequencySamplingThread.frequencyHz))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(5, () -> driveSpark.configure(
                driveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));
        SparkUtil.tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(moduleConfig.turnInverted())
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(moduleConfig.turnCurrentLimit())
                .voltageCompensation(12.0);
        turnConfig
                .encoder
                .positionConversionFactor(2 * Math.PI / moduleConfig.turnGearRatio()) // Rotor Rotations -> Wheel Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / moduleConfig.turnGearRatio()) // Rotor RPM -> Wheel Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0, 2 * Math.PI);
        moduleConfig.turnGains().applySparkPID(turnConfig.closedLoop, ClosedLoopSlot.kSlot0);
        turnConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / HighFrequencySamplingThread.frequencyHz))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(5, () -> turnSpark.configure(
                turnConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));

        // Configure CANCoder
        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(-absoluteEncoderOffsetRad);
        cancoderConfig.MagnetSensor.SensorDirection =
                moduleConfig.encoderInverted()
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

        turnAbsolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);
        ParentDevice.optimizeBusUtilizationForAll(cancoder);

        SparkCANcoderHelper.resetTurnSpark(turnEncoder, turnAbsolutePosition, cancoderCanID);

        // Create odometry queues
        timestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        drivePositionQueue = HighFrequencySamplingThread.get().registerSparkSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = HighFrequencySamplingThread.get().registerSparkSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        SparkUtil.ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        SparkUtil.ifOk(
                driveSpark,
                new DoubleSupplier[]{driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]
        );
        SparkUtil.ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        SparkUtil.ifOk(driveSpark, driveSpark::getMotorTemperature, (value) -> inputs.driveTemperatureCelsius = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        // Update turn inputs
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPositionRad = value
        );
        SparkUtil.ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        SparkUtil.ifOk(
                turnSpark,
                new DoubleSupplier[]{turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]
        );
        SparkUtil.ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        SparkUtil.ifOk(turnSpark, turnSpark::getMotorTemperature, (value) -> inputs.turnTemperatureCelsius = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        // Turn cancoder
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnAbsoluteEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream().mapToDouble((Double value) -> value).toArray();

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDrivePIDF(PIDF newGains) {
        System.out.println("Setting drive gains");
        driveFF = newGains.toSimpleFF();
        var newConfig = new SparkMaxConfig();
        newGains.applySparkPID(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        SparkUtil.tryUntilOkAsync(5, () -> driveSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setTurnPIDF(PIDF newGains) {
        System.out.println("Setting turn gains");
        var newConfig = new SparkMaxConfig();
        newGains.applySparkPID(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        SparkUtil.tryUntilOkAsync(5, () -> turnSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.tryUntilOkAsync(5, () -> driveSpark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.tryUntilOkAsync(5, () -> turnSpark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        var ffVolts = driveFF.calculate(velocityRadPerSec);
        driveController.setReference(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(double positionRad) {
        double setpoint = MathUtil.inputModulus(positionRad, 0.0, 2 * Math.PI);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
