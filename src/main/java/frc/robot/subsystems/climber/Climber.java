package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.climber.ClimberConstants.*;

public class Climber extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ClimberIO io = createIO();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(0),
        TOWARDS_ROBOT(12),
        AWAY_FROM_ROBOT(-12);

        public final double volts;
    }

    @Getter
    private Goal goal = Goal.IDLE;

    @AutoLogOutput(key = "Climber/HasZeroed")
    private boolean hasZeroed = false;

    private final Alert disconnectedAlert = new Alert("Climber motor motor is disconnected.", Alert.AlertType.kError);
    private final Alert temperatureAlert = new Alert("Climber motor temperature is high.", Alert.AlertType.kWarning);
    private final Alert notZeroedAlert = new Alert("Climber is not zeroed.", Alert.AlertType.kWarning);
    private final Alert hitLimitAlert = new Alert("Climber has hit its limit.", Alert.AlertType.kWarning);
    private final Alert bypassLimitsAlert = new Alert("Climber limits are being bypassed.", Alert.AlertType.kWarning);

    private static Climber instance;

    public static Climber get() {
        if (instance == null)
            synchronized (Climber.class) {
                instance = new Climber();
            }

        return instance;
    }

    private Climber() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);

        disconnectedAlert.set(!inputs.connected);
        temperatureAlert.set(inputs.temperatureCelsius > 35);

        bypassLimitsAlert.set(operatorDashboard.bypassClimberLimits.get());

        robotMechanism.climber.ligament.setAngle(Units.radiansToDegrees(inputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        Logger.recordOutput("Climber/Goal", goal);
        boolean canRun = goal.volts > 0
                ? inputs.positionRad < upperLimitRad
                : inputs.positionRad > lowerLimitRad;
        hitLimitAlert.set(!canRun);
        boolean shouldStop = !canRun && !operatorDashboard.bypassClimberLimits.get();
        if (shouldStop || DriverStation.isDisabled()) {
            Logger.recordOutput("Climber/Running", false);
            io.setOpenLoop(0);
        } else {
            Logger.recordOutput("Climber/Running", true);
            io.setOpenLoop(goal.volts);
        }

        // Check force zero and zero if needed
        var forceZero = operatorDashboard.forceZeroClimber.get();
        if (forceZero) {
            io.setEncoder(initialPositionRad);
            hasZeroed = true;
            // Turn off the toggle instantly so it's like a button
            operatorDashboard.forceZeroClimber.set(false);
        }
        notZeroedAlert.set(!hasZeroed);
    }

    public Command idle() {
        return startIdle(() -> goal = Goal.IDLE);
    }

    public Command towardsRobot() {
        return startIdle(() -> goal = Goal.TOWARDS_ROBOT);
    }

    public Command awayFromRobot() {
        return startIdle(() -> goal = Goal.AWAY_FROM_ROBOT);
    }
}