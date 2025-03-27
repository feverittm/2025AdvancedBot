package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.climber.ClimberConstants.gearRatio;
import static frc.robot.subsystems.climber.ClimberConstants.initialPositionRad;

public class ClimberIOSim extends ClimberIO {
    private final DCMotor motor = DCMotor.getKrakenX60(1);
    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, .002, gearRatio),
            motor,
            0.004,
            0.0
    );
    private double appliedVolts;

    public ClimberIOSim() {
        motorSim.setState(initialPositionRad, 0);
    }

    @Override
    public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting climber brake mode to " + enable);
    }

    @Override
    public void setOpenLoop(double output) {
        appliedVolts = output;
    }

    @Override
    public void setEncoder(double positionRad) {
        motorSim.setState(positionRad, motorSim.getAngularVelocityRadPerSec());
    }
}
