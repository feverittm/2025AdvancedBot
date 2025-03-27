package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public void updateInputs(ClimberIOInputs inputs) {
    }

    public void setBrakeMode(boolean enable) {
    }

    /**
     * Run the motor at the specified open loop value.
     */
    public void setOpenLoop(double output) {
    }

    /** Set the encoder of the motor to the specified position. Used for zeroing */
    public void setEncoder(double positionRad) {
    }
}