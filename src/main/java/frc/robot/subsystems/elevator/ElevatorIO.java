package frc.robot.subsystems.elevator;

import frc.robot.util.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leaderConnected = false;
        public double leaderPositionRad = 0.0;
        public double leaderVelocityRadPerSec = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderTemperatureCelsius = 0.0;

        public boolean followerConnected = false;
        public double followerPositionRad = 0.0;
        public double followerVelocityRadPerSec = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double followerTemperatureCelsius = 0.0;

        public boolean limitSwitchTriggered = false;
    }

    public void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setPIDF(PIDF newGains) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    public void setBrakeMode(boolean enable) {
    }

    /**
     * Set the emergency stopped value. IO layers should expect this only to be called when the value changes.
     * If true, the IO layer should ignore all commands and stop the motor until set back to false.
     */
    public void setEmergencyStopped(boolean emergencyStopped) {
    }

    /**
     * Run the motor at the specified open loop value.
     */
    public void setOpenLoop(double output) {
    }

    /**
     * Run the motor to the specified position.
     * Should use feedback to go to the specified position and use feedforward to go to the specified velocity
     */
    public void setClosedLoop(double positionRad, double velocityRadPerSec) {
    }

    /** Set the encoder of the elevator to the specified position. Used for zeroing */
    public void setEncoder(double positionRad) {
    }
}