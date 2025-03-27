//package frc.robot.subsystems.coralintake;
//
//import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
//import org.littletonrobotics.junction.Logger;
//
//import static frc.robot.subsystems.coralintake.CoralIntakeConstants.*;
//
//public class PivotIOSim extends PivotIO {
//    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
//            DCMotor.getKrakenX60(1),
//            pivotConfig.motorGearRatio(),
//            0.1456967969 + Units.lbsToKilograms(15.522) * Math.pow(Units.inchesToMeters(17.0502529), 2),
//            Units.inchesToMeters(17.0502529),
//            0.12833586,
//            1.353,
//            true,
//            1.353,
//            0.00001, // position
//            0.00001 // velocity, probably
//    );
//    private final ProfiledPIDController pid = pivotConfig.gains().toProfiledPID(new TrapezoidProfile.Constraints(
//            pivotMaxVelocityRadPerSec,
//            pivotMaxAccelerationRadPerSecSquared
//    ));
//    private final ArmFeedforward ff = pivotConfig.gains().toArmFF();
//
//    private boolean closedLoop = true;
//    private double appliedVolts;
//
//    public PivotIOSim() {
//    }
//
//    @Override
//    public void updateInputs(PivotIOInputs inputs) {
//        if (closedLoop) {
//            appliedVolts = pid.calculate(armSim.getAngleRads())
//                    + ff.calculate(armSim.getAngleRads(), pid.getSetpoint().velocity);
//            Logger.recordOutput("CoralIntake/Pivot/SetpointVelocityRadPerSec", pid.getSetpoint().velocity);
//        } else {
//            pid.reset(new TrapezoidProfile.State(armSim.getAngleRads(), armSim.getVelocityRadPerSec()));
//        }
//
//        armSim.setInputVoltage(appliedVolts);
//
//        armSim.update(0.02);
//
//        inputs.connected = true;
//        inputs.positionRad = armSim.getAngleRads();
//        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
//        inputs.appliedVolts = appliedVolts;
//        inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
//    }
//
//    @Override
//    public void setOpenLoop(double output) {
//        closedLoop = false;
//        appliedVolts = output;
//    }
//
//    @Override
//    public void setClosedLoop(double positionRad) {
//        closedLoop = true;
//        pid.setGoal(new TrapezoidProfile.State(positionRad, 0));
//    }
//}
