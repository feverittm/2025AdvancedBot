package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.PhoenixUtil.tryUntilOkAsync;

public class ClimberIOTalonFX extends ClimberIO {
    private final TalonFX talon;

    private final TalonFXConfiguration talonConfig;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> temperatureCelsius;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public ClimberIOTalonFX(int canID, boolean inverted) {
        talon = new TalonFX(canID, Constants.CANivore.busName);

        // Configure drive motor
        talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
        talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimitAmps;
        talonConfig.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.MotorOutput.Inverted =
                inverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        tryUntilOk(5, () -> talon.setPosition(Units.radiansToRotations(initialPositionRad), 0.25));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        currentAmps = talon.getStatorCurrent();
        temperatureCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVolts,
                currentAmps,
                temperatureCelsius
        );
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Refresh all signals
        var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, temperatureCelsius);

        // Update drive inputs
        inputs.connected = connectedDebounce.calculate(status.isOK());
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        talonConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    }

    @Override
    public void setOpenLoop(double output) {
        talon.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void setEncoder(double positionRad) {
        tryUntilOkAsync(5, () -> talon.setPosition(Units.radiansToRotations(positionRad), 0.25));
    }
}
