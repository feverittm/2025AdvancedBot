package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.network.LoggedNetworkBooleanExt;
import frc.robot.util.network.LoggedNetworkNumberExt;
import frc.robot.util.subsystem.VirtualSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.IntFunction;

public class OperatorDashboard extends VirtualSubsystem {
    private static final String prefix = "/OperatorDashboard/";

    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt coralStuckInRobotMode = new LoggedNetworkBooleanExt(prefix + "CoralStuckInRobotMode", false);
    public final LoggedNetworkBooleanExt manualScoring = new LoggedNetworkBooleanExt(prefix + "ManualScoring", false);
    public final LoggedNetworkBooleanExt ignoreEndEffectorBeamBreak = new LoggedNetworkBooleanExt(prefix + "IgnoreEndEffectorBeamBreak", false);
    public final LoggedNetworkBooleanExt disableInterpolateAutoAlign = new LoggedNetworkBooleanExt(prefix + "DisableInterpolateAutoAlign", false);

    public final LoggedNetworkBooleanExt forceZeroClimber = new LoggedNetworkBooleanExt(prefix + "ForceZeroClimber", false);
    public final LoggedNetworkBooleanExt bypassClimberLimits = new LoggedNetworkBooleanExt(prefix + "BypassClimberLimits", false);

    public final LoggedNetworkBooleanExt elevatorEStop = new LoggedNetworkBooleanExt(prefix + "ElevatorEStop", false);
    public final LoggedNetworkBooleanExt useRealElevatorState = new LoggedNetworkBooleanExt(prefix + "UseRealElevatorState", false);
    public final LoggedNetworkBooleanExt forceZeroElevator = new LoggedNetworkBooleanExt(prefix + "ForceZeroElevator", false);
    public final LoggedNetworkNumberExt elevatorOffsetMeters = new LoggedNetworkNumberExt(prefix + "ElevatorOffsetMeters", 0);

    private final Map<ReefZoneSide, LoggedNetworkBooleanExt> reefZoneSides = generateTogglesForEnum("ReefZoneSides", ReefZoneSide.values());
    private final Map<LocalReefSide, LoggedNetworkBooleanExt> localReefSides = generateTogglesForEnum("LocalReefSides", LocalReefSide.values());
    private final Map<CoralScoringLevel, LoggedNetworkBooleanExt> coralScoringLevels = generateTogglesForEnum("CoralScoringLevels", CoralScoringLevel.values());

    @Getter
    private ReefZoneSide selectedReefZoneSide = ReefZoneSide.LeftFront;
    @Getter
    private LocalReefSide selectedLocalReefSide = LocalReefSide.Left;
    @Getter
    private CoralScoringLevel selectedCoralScoringLevel = CoralScoringLevel.L4;

    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);
    private final Alert coralStuckInRobotModeAlert = new Alert("Coral stuck in robot mode is enabled.", Alert.AlertType.kWarning);
    private final Alert manualScoringAlert = new Alert("Manual scoring is enabled.", Alert.AlertType.kWarning);
    private final Alert ignoreEndEffectorBeamBreakAlert = new Alert("Ignore end effector beam break is enabled.", Alert.AlertType.kWarning);

    private final OperatorKeypad operatorKeypad = new OperatorKeypad();
    private final Alert operatorKeypadDisconnectedAlert = new Alert("Operator keypad is not connected!", Alert.AlertType.kWarning);

    private static OperatorDashboard instance;

    public static OperatorDashboard get() {
        if (instance == null)
            synchronized (OperatorDashboard.class) {
                instance = new OperatorDashboard();
            }

        return instance;
    }

    private OperatorDashboard() {
    }

    @Override
    public void periodicBeforeCommands() {
        // Note - we only handle alerts for general overrides.
        // So elevator and climber toggles are handled in their respective subsystems
        coastOverrideAlert.set(coastOverride.get());
        coralStuckInRobotModeAlert.set(coralStuckInRobotMode.get());
        manualScoringAlert.set(manualScoring.get());
        ignoreEndEffectorBeamBreakAlert.set(ignoreEndEffectorBeamBreak.get());

        if (operatorKeypad.isConnected()) {
            operatorKeypadDisconnectedAlert.set(false);

            ReefZoneSide newReefZoneSide = operatorKeypad.getReefZoneSide();
            if (newReefZoneSide != null) selectedReefZoneSide = newReefZoneSide;
            updateToggles(reefZoneSides, selectedReefZoneSide);

            CoralScoringLevel newCoralScoringLevel = operatorKeypad.getCoralScoringLevel();
            if (newCoralScoringLevel != null) selectedCoralScoringLevel = newCoralScoringLevel;
            updateToggles(coralScoringLevels, selectedCoralScoringLevel);

            LocalReefSide newLocalReefSide = operatorKeypad.getLocalReefSide();
            if (newLocalReefSide != null) selectedLocalReefSide = newLocalReefSide;
            updateToggles(localReefSides, selectedLocalReefSide);
        } else {
            operatorKeypadDisconnectedAlert.set(true);

            handleEnumToggles(reefZoneSides, selectedReefZoneSide, selectNew -> selectedReefZoneSide = selectNew);
            handleEnumToggles(localReefSides, selectedLocalReefSide, selectNew -> selectedLocalReefSide = selectNew);
            handleEnumToggles(coralScoringLevels, selectedCoralScoringLevel, selectNew -> selectedCoralScoringLevel = selectNew);
        }

    }

    public Elevator.Goal getCoralScoringElevatorGoal() {
        return switch (selectedCoralScoringLevel) {
            case L1 -> Elevator.Goal.SCORE_L1;
            case L2 -> Elevator.Goal.SCORE_L2;
            case L3 -> Elevator.Goal.SCORE_L3;
            case L4 -> Elevator.Goal.SCORE_L4;
        };
    }

    public Elevator.Goal getAlgaeDescoringElevatorGoal() {
        return switch (selectedReefZoneSide) {
            case LeftFront, RightFront, MiddleBack -> Elevator.Goal.DESCORE_L2;
            case MiddleFront, RightBack, LeftBack -> Elevator.Goal.DESCORE_L3;
        };
    }

    @RequiredArgsConstructor
    public enum ReefZoneSide {
        LeftFront(0),
        MiddleFront(1),
        RightFront(2),
        RightBack(3),
        MiddleBack(4),
        LeftBack(5);

        public final int aprilTagOffset;
    }

    @RequiredArgsConstructor
    public enum LocalReefSide {
        Left,
        Right,
    }

    public enum CoralScoringLevel {
        L1,
        L2,
        L3,
        L4,
    }

    private static <E extends Enum<E>> void updateToggles(
            Map<E, LoggedNetworkBooleanExt> map,
            E currentlySelected
    ) {
        LoggedNetworkBooleanExt toggle = map.get(currentlySelected);
        // Set the corresponding toggle to true
        toggle.set(true);
        // Set the rest to false
        for (var entry1 : map.entrySet()) {
            if (entry1.getKey() != currentlySelected) {
                entry1.getValue().set(false);
            }
        }
    }

    private static <E extends Enum<E>> void handleEnumToggles(
            Map<E, LoggedNetworkBooleanExt> map,
            E currentlySelected,
            Consumer<E> select
    ) {
        // If none are toggled
        if (map.values().stream().noneMatch(LoggedNetworkBooleanExt::get)) {
            // Enable the last selected one
            map.get(currentlySelected).set(true);
        } else {
            // Otherwise, look for changes in the toggles
            for (var entry : map.entrySet()) {
                LoggedNetworkBooleanExt toggle = entry.getValue();
                // If it's toggled
                if (toggle.get()) {
                    E key = entry.getKey();
                    // If it wasn't already selected
                    if (key != currentlySelected) {
                        // Select the new value
                        select.accept(key);
                        // Set the rest to false
                        for (var entry1 : map.entrySet()) {
                            if (entry1.getKey() != key) {
                                entry1.getValue().set(false);
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    private static <E extends Enum<E>> Map<E, LoggedNetworkBooleanExt> generateTogglesForEnum(String name, E[] values) {
        return Map.ofEntries(
                Arrays.stream(values)
                        .map(side -> Map.entry(
                                side,
                                new LoggedNetworkBooleanExt(prefix + name + "/" + side.name(), false)
                        ))
                        .toArray((IntFunction<Map.Entry<E, LoggedNetworkBooleanExt>[]>) Map.Entry[]::new)
        );
    }

    private static class OperatorKeypad {
        private final GenericHID hid = new GenericHID(1);

        public boolean isConnected() {
            return hid.isConnected();
        }

        public ReefZoneSide getReefZoneSide() {
            if (hid.getRawButton(1)) return ReefZoneSide.LeftFront;
            if (hid.getRawButton(2)) return ReefZoneSide.MiddleFront;
            if (hid.getRawButton(3)) return ReefZoneSide.RightFront;
            if (hid.getRawButton(4)) return ReefZoneSide.RightBack;
            if (hid.getRawButton(5)) return ReefZoneSide.MiddleBack;
            if (hid.getRawButton(6)) return ReefZoneSide.LeftBack;
            return null;
        }

        public CoralScoringLevel getCoralScoringLevel() {
            if (hid.getRawButton(7)) return CoralScoringLevel.L1;
            if (hid.getRawButton(8)) return CoralScoringLevel.L2;
            if (hid.getRawButton(9)) return CoralScoringLevel.L3;
            if (hid.getRawButton(10)) return CoralScoringLevel.L4;
            return null;
        }

        public LocalReefSide getLocalReefSide() {
            if (hid.getRawButton(11)) return LocalReefSide.Left;
            if (hid.getRawButton(12)) return LocalReefSide.Right;
            return null;
        }
    }
}
