package frc.robot.subsystems.rollers;

import frc.robot.util.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(RollersIOInputs inputs) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setPositionPIDF(PIDF newGains) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setVelocityPIDF(PIDF newGains) {
    }

    /**
     * Enable or disable brake mode on the motor.
     */
    public void setBrakeMode(boolean enable) {
    }

    /**
     * Run the motor at the specified open loop value.
     */
    public void setOpenLoop(double output) {
    }

    /**
     * Run the motor to the specified velocity.
     */
    public void setVelocity(double velocityRadPerSec) {
    }

    /**
     * Run the motor to the specified rotation.
     */
    public void setPosition(double positionRad) {
    }
}
