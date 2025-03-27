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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.HighFrequencySamplingThread;

import java.util.Queue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

/**
 * IO implementation for Pigeon 2.
 */
public class GyroIOPigeon2 extends GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOPigeon2(int canID) {
        pigeon = new Pigeon2(canID, Constants.CANivore.busName);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        tryUntilOk(5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()));
        tryUntilOk(5, () -> pigeon.getConfigurator().setYaw(0.0));
        pigeon.optimizeBusUtilization();

        yaw.setUpdateFrequency(HighFrequencySamplingThread.frequencyHz);
        yawVelocity.setUpdateFrequency(50.0);

        yawTimestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        yawPositionQueue = HighFrequencySamplingThread.get().registerPhoenixSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRad =
                yawPositionQueue.stream()
                        .mapToDouble(Units::degreesToRadians)
                        .toArray();

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
