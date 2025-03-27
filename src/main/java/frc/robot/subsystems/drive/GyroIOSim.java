package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroIOSim extends GyroIO {
    private final GyroSimulation gyroSimulation = ModuleIOSim.driveSimulation.getGyroSimulation();

    public GyroIOSim() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPositionRad = gyroSimulation.getGyroReading().getRadians();
        inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

        inputs.odometryYawTimestamps = ModuleIOSim.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositionsRad = Arrays.stream(gyroSimulation.getCachedGyroReadings())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }
}
