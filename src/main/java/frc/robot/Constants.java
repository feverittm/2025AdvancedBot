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

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.controller.CommandSteamInputController;

import java.util.function.Function;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final boolean tuningMode = false;

    public static final RobotIdentity identity = RobotIdentity.determine();

    public static final Mode mode = RobotBase.isReal()
            ? Mode.REAL
            : (Simulation.shouldReplay ? Mode.REPLAY : Mode.SIM);

    public static final boolean isReplay = mode == Mode.REPLAY;

    public enum Mode {
        /**
         * Real robot
         */
        REAL,
        /**
         * Simulation
         */
        SIM,
        /**
         * Log replay
         */6
        REPLAY
    }

    public static final class Simulation {
        /**
         * Set this to a RobotIdentity to replay using a log from that robot.
         * If replayIdentity is null, log replay will not run and simulation will run like normal.
         */
        public static final RobotIdentity replayIdentity = null;
        @SuppressWarnings("ConstantValue")
        public static final boolean shouldReplay = RobotBase.isSimulation() && replayIdentity != null; // Don't modify please!

        /**
         * If true, replay will run as fast as your computer can go and log to a log file instead of
         * NetworkTables. You will have to open the log file to see anything.
         */
        public static final boolean replayRunAsFastAsPossible = true;

        public static final Function<Integer, CommandXboxController> simController = (port) -> {
            if (System.getProperty("os.name").contains("Mac OS X")) {
                return new CommandSteamInputController(port);
//                return new CommandNintendoSwitchProController(port);
            }

            return new CommandXboxController(port);
        };
    }

    public static final class CANivore {
        public static final String busName = "*"; // the canivore is called electrical_problem, but using * is better because it will select any canivore it sees
        public static final boolean isCANFD = switch (Constants.identity) {
            case COMPBOT -> new CANBus(busName).isNetworkFD();
            case ALPHABOT, SIMBOT -> false;
        };
    }
}
