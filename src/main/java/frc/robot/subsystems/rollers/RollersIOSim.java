package frc.robot.subsystems.rollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PIDF;

public class RollersIOSim extends RollersIO {
    private final DCMotorSim motorSim;
    private PIDController positionPid;
    private PIDController velocityPid;
    private SimpleMotorFeedforward velocityFeedforward;

    private double appliedVolts;
    private boolean closedLoop = true;
    private boolean positionControl = false;
    private double ffVolts;


    // If using SysID values, kA and kV are the gains returned from SysID, in volts/(rad/sec) or volts/(rad/sec^2)
    public RollersIOSim(RollersConfig config, DCMotor motor, double kV, double kA) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kV, kA),
                motor,
                0.004
        );

        positionPid = config.positionGains().toPID();
        velocityFeedforward = config.velocityGains().toSimpleFF();
        velocityPid = config.velocityGains().toPID();
    }

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public RollersIOSim(RollersConfig config, double JKgMetersSquared, DCMotor motor) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, config.gearRatio()),
                motor,
                0.004,
                0.0
        );

        velocityFeedforward = config.velocityGains().toSimpleFF();
        positionPid = config.positionGains().toPID();
        velocityPid = config.velocityGains().toPID();
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        if (closedLoop) {
            if (positionControl) {
                appliedVolts = positionPid.calculate(motorSim.getAngularPositionRad());
            } else {
                appliedVolts = velocityPid.calculate(motorSim.getAngularVelocityRadPerSec()) + ffVolts;
            }
        }

        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting roller position gains");
        positionPid = newGains.toPID();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting roller velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        velocityPid = newGains.toPID();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting rollers brake mode to " + enable);
    }

    @Override
    public void setOpenLoop(double output) {
        appliedVolts = output;
        closedLoop = false;
    }

    @Override
    public void setPosition(double positionRad) {
        closedLoop = true;
        positionControl = true;
        positionPid.setSetpoint(positionRad);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        closedLoop = true;
        positionControl = false;
        ffVolts = velocityFeedforward.calculate(velocityRadPerSec);
        velocityPid.setSetpoint(velocityRadPerSec);
    }
}
