package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class SparkCANcoderHelper {
    private static final Alert turnRelativeEncoderNotResetAlert = new Alert("One or more drive modules has not successfully reset their relative turn encoder", Alert.AlertType.kError);
    private static final ArrayList<Integer> resetFailedCANcoderIDs = new ArrayList<>();

    public static void resetTurnSpark(
            RelativeEncoder turnEncoder,
            StatusSignal<Angle> turnAbsolutePosition,
            int cancoderCanID
    ) {
        // Reset turn spark
        // Not the prettiest but doing it now is better than in updateInputs
        try {
            // Wait for cancoder status signal to use new offset
            // TODO: do this better
            Thread.sleep(500);
        } catch (InterruptedException ignored) {
        }
        var successful = false;
        // 15 attempts because this is really important
        for (int i = 0; i < 15; i++) {
            var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
            var turnEncoderConnected = turnEncoderStatus.isOK();
            if (turnEncoderConnected) {
                var absolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
                if (absolutePositionRad != 0) {
                    SparkUtil.sparkStickyFault = false;
                    SparkUtil.tryUntilOk(5, () -> turnEncoder.setPosition(absolutePositionRad));
                    if (!SparkUtil.sparkStickyFault) {
                        System.out.printf("Drive module with cancoder ID %d setting initial position of turn relative encoder to %s%n", cancoderCanID, absolutePositionRad);
                        successful = true;
                        break;
                    }
                }
            }
            System.out.printf("Drive module with cancoder ID %d FAILED on attempt %d to set initial position of turn relative encoder (connected: %s, sparkStickyFault: %s)%n", cancoderCanID, i + 1, turnEncoderConnected, SparkUtil.sparkStickyFault);
        }
        if (!successful) {
            System.out.printf("Drive module with cancoder ID %d GAVE UP setting initial position of turn relative encoder%n", cancoderCanID);
            resetFailedCANcoderIDs.add(cancoderCanID);
            Logger.recordOutput("Drive/SparkResetFailedCANcoderIDs", resetFailedCANcoderIDs.stream().mapToInt((val) -> val).toArray());
            turnRelativeEncoderNotResetAlert.set(true);
        }
    }
}
