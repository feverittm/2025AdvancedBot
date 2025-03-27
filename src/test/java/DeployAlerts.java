import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;

import javax.swing.*;

class DeployAlerts {
    public static void main(String[] args) {
        if (Constants.tuningMode)
            confirmDeploy("Tuning mode is enabled.");

        if (DriveConstants.disableDriving)
            confirmDeploy("Disable driving is enabled.");
    }

    private static void confirmDeploy(String message) {
        if (isNo(message + " Are you sure you want to deploy?")) {
            throw new RuntimeException("Cancelled deploy");
        }
    }

    private static boolean isNo(String message) {
        return JOptionPane.showConfirmDialog(
                null,
                message,
                "Deploy Alerts",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.WARNING_MESSAGE
        ) != 0; // 0 = yes, 1 = no, -1 = close window
    }
}
