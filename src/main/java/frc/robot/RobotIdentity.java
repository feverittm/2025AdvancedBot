package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

public enum RobotIdentity {
    COMPBOT("0329F35E"),
    ALPHABOT(""),
    SIMBOT(null);

    public final String serialNumber;

    RobotIdentity(String serialNumber) {
        this.serialNumber = serialNumber;
    }

    private static final Alert unmatchedSerialNumberAlert = new Alert("Serial number did not match any robot identities. Assuming COMPBOT.", Alert.AlertType.kWarning);

    public static RobotIdentity determine() {
        if (RobotBase.isReal()) {
            final var serialNumber = RobotController.getSerialNumber();

            Logger.recordMetadata("SerialNumber", serialNumber);

            if (serialNumber.equals(COMPBOT.serialNumber))
                return COMPBOT;

            if (serialNumber.equals(ALPHABOT.serialNumber))
                return ALPHABOT;

            String msg = "Serial number " + serialNumber + " did not match any robot identities. Assuming COMPBOT.";
            unmatchedSerialNumberAlert.setText(msg);
            System.out.println(msg);
            unmatchedSerialNumberAlert.set(true);
            return COMPBOT;
        } else {
            if (Constants.Simulation.shouldReplay)
                return Constants.Simulation.replayIdentity;

            return RobotIdentity.SIMBOT;
        }
    }
}