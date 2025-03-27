package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static frc.robot.subsystems.elevator.ElevatorConstants.radToMeters;
import static frc.robot.subsystems.elevator.ElevatorConstants.sprocketRadiusMeters;
import static org.junit.jupiter.api.Assertions.assertEquals;

class ElevatorConstantsTests {
    static final double delta = 1e-2;

    @Test
    void radToMetersTest() {
        var distancePerStagePerRotation = sprocketRadiusMeters * 2 * Math.PI;
        for (int rotations = 0; rotations < 10; rotations++) {
            assertEquals(rotations * distancePerStagePerRotation * 3, radToMeters(Units.rotationsToRadians(rotations)), delta);
        }
    }
}
