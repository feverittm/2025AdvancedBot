// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import frc.robot.util.HighFrequencySamplingThread;

import java.util.Queue;

/**
 * IO implementation for NavX.
 */
public class GyroIONavX extends GyroIO {
    private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) HighFrequencySamplingThread.frequencyHz);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX() {
        yawTimestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        yawPositionQueue = HighFrequencySamplingThread.get().registerGenericSignal(navX::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPositionRad = Units.degreesToRadians(-navX.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
        inputs.temperatureCelsius = navX.getTempC();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRad =
                yawPositionQueue.stream()
                        .mapToDouble((Double value) -> Units.degreesToRadians(-value))
                        .toArray();
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
