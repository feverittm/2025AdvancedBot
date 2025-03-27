package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PIDF;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;
import static frc.robot.subsystems.elevator.ElevatorConstants.gearRatio;
import static frc.robot.util.SparkUtil.*;

public class ElevatorIOSparkMax extends ElevatorIO {
    // Hardware objects
    private final SparkMax leadMotor;
    private final SparkMax followMotor;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followEncoder;
    private final SparkMaxConfig leadConfig;
    private final SparkMaxConfig followConfig;

    private final DigitalInput limitSwitch;

    // Closed loop controllers
    private final SparkClosedLoopController controller;
    private ElevatorFeedforward ff = gains.toElevatorFF();

    // Connection debouncers
    private final Debouncer leadConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followConnectedDebounce = new Debouncer(0.5);

    private boolean emergencyStopped = false;

    /** Used when calculating feedforward */
    private double lastVelocitySetpointRadPerSec = 0;

    public ElevatorIOSparkMax(
            int leadCanID,
            int followCanID,
            int limitSwitchID,
            boolean leaderInverted
    ) {
        leadMotor = new SparkMax(leadCanID, SparkLowLevel.MotorType.kBrushless);
        followMotor = new SparkMax(followCanID, SparkLowLevel.MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        followEncoder = followMotor.getEncoder();
        controller = leadMotor.getClosedLoopController();

        limitSwitch = new DigitalInput(limitSwitchID);

        // Configure motors
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();

        leadConfig
                .inverted(leaderInverted)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);
        leadConfig
                .encoder
                .positionConversionFactor(2 * Math.PI / gearRatio) // Rotor Rotations -> Drum Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / gearRatio) // Rotor RPM -> Drum Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        leadConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        gains.applySparkPID(leadConfig.closedLoop, ClosedLoopSlot.kSlot0);
        leadConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        followConfig.apply(leadConfig).follow(leadMotor, true);

        tryUntilOk(5, () -> leadMotor.configure(
                leadConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> followMotor.configure(
                followConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> leadEncoder.setPosition(0.0));
        tryUntilOk(5, () -> followEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update lead inputs
        sparkStickyFault = false;
        ifOk(leadMotor, leadEncoder::getPosition, (value) -> inputs.leaderPositionRad = value);
        ifOk(leadMotor, leadEncoder::getVelocity, (value) -> inputs.leaderVelocityRadPerSec = value);
        ifOk(
                leadMotor,
                new DoubleSupplier[]{leadMotor::getAppliedOutput, leadMotor::getBusVoltage},
                (values) -> inputs.leaderAppliedVolts = values[0] * values[1]
        );
        ifOk(leadMotor, leadMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
        ifOk(leadMotor, leadMotor::getMotorTemperature, (value) -> inputs.leaderTemperatureCelsius = value);
        inputs.leaderConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

        // Update follow inputs
        sparkStickyFault = false;
        ifOk(followMotor, followEncoder::getPosition, (value) -> inputs.followerPositionRad = value);
        ifOk(followMotor, followEncoder::getVelocity, (value) -> inputs.followerVelocityRadPerSec = value);
        ifOk(
                followMotor,
                new DoubleSupplier[]{followMotor::getAppliedOutput, followMotor::getBusVoltage},
                (values) -> inputs.followerAppliedVolts = values[0] * values[1]
        );
        ifOk(followMotor, followMotor::getOutputCurrent, (value) -> inputs.followerCurrentAmps = value);
        ifOk(followMotor, followMotor::getMotorTemperature, (value) -> inputs.followerTemperatureCelsius = value);
        inputs.followerConnected = followConnectedDebounce.calculate(!sparkStickyFault);

        inputs.limitSwitchTriggered = !limitSwitch.get();
    }

    @Override
    public void setPIDF(PIDF newGains) {
        System.out.println("Setting elevator gains");
        ff = newGains.toElevatorFF();
        var newConfig = new SparkMaxConfig();
        newGains.applySparkPID(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        tryUntilOkAsync(5, () -> leadMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> leadMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setEmergencyStopped(boolean emergencyStopped) {
        this.emergencyStopped = emergencyStopped;
        if (emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leadMotor.setVoltage(0);
        }
    }

    @Override
    public void setOpenLoop(double output) {
        if (!emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leadMotor.setVoltage(output);
        }
    }

    @Override
    public void setClosedLoop(double positionRad, double velocityRadPerSec) {
        if (!emergencyStopped) {
            var ffVolts = ff.calculateWithVelocities(lastVelocitySetpointRadPerSec, velocityRadPerSec);
            lastVelocitySetpointRadPerSec = velocityRadPerSec;
            controller.setReference(
                    positionRad,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    ffVolts,
                    SparkClosedLoopController.ArbFFUnits.kVoltage
            );
        }
    }

    @Override
    public void setEncoder(double positionRad) {
        tryUntilOkAsync(5, () -> leadEncoder.setPosition(positionRad));
        tryUntilOkAsync(5, () -> followEncoder.setPosition(positionRad));
    }
}
