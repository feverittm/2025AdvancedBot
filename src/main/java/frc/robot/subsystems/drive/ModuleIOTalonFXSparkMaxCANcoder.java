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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.HighFrequencySamplingThread;
import frc.robot.util.PIDF;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;

/**
 * Module IO implementation for Talon FX drive motor controller, Spark MAX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXSparkMaxCANcoder extends ModuleIO {
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    // Hardware objects
    private final TalonFX driveTalon;
    private final SparkMax turnSpark;
    private final RelativeEncoder turnEncoder;
    private final CANcoder cancoder;

    private final TalonFXConfiguration driveConfig;
    private final SparkMaxConfig turnConfig;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
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
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFXSparkMaxCANcoder(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        driveTalon = new TalonFX(driveCanID, Constants.CANivore.busName);
        turnSpark = new SparkMax(turnCanID, SparkLowLevel.MotorType.kBrushless);
        cancoder = new CANcoder(cancoderCanID, Constants.CANivore.busName);
        turnEncoder = turnSpark.getEncoder();
        turnController = turnSpark.getClosedLoopController();

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
        PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(moduleConfig.turnInverted())
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
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
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
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
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));

        // Configure CANCoder
        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(-absoluteEncoderOffsetRad);
        cancoderConfig.MagnetSensor.SensorDirection =
                moduleConfig.encoderInverted()
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

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
        turnPositionQueue = HighFrequencySamplingThread.get().registerSparkSignal(turnSpark, turnEncoder::getPosition);

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(HighFrequencySamplingThread.frequencyHz, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrentAmps,
                driveTemperatureCelsius,
                turnAbsolutePosition
        );
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, cancoder);

        SparkCANcoderHelper.resetTurnSpark(turnEncoder, turnAbsolutePosition, cancoderCanID);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrentAmps, driveTemperatureCelsius);
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
        inputs.driveTemperatureCelsius = driveTemperatureCelsius.getValueAsDouble();

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

        // Update absolute encoder
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnAbsoluteEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream()
                .mapToDouble((Double value) -> value)
                .toArray();

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDrivePIDF(PIDF newGains) {
        System.out.println("Setting drive gains");
        driveConfig.Slot0 = Slot0Configs.from(newGains.toPhoenix());
        PhoenixUtil.tryUntilOkAsync(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
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
        driveConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        PhoenixUtil.tryUntilOkAsync(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        SparkUtil.tryUntilOkAsync(5, () -> turnSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
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
        turnSpark.setVoltage(output);
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
        double setpoint = MathUtil.inputModulus(positionRad, 0.0, 2 * Math.PI);
        turnController.setReference(setpoint, SparkBase.ControlType.kPosition);
    }
}
