package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
  // TODO: SET TO ACTUAL VALUES WHEN CAD IS FINISHED
  // Taken from Citrus circuits 2024 CAD
  SingleJointedArmSim intakePivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          IntakePivotSubsystem.PIVOT_RATIO,
          0.07,
          edu.wpi.first.math.util.Units.inchesToMeters(108.5),
          0.0,
          0.5,
          true,
          0.0);

  // TODO: TUNE
  ProfiledPIDController pivotPid =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));
  // TODO: TUNE
  ArmFeedforward pivotFf = new ArmFeedforward(0.0, 0.0, 0.0);

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    intakePivotSim.update(0.02);

    inputs.pivotPosition = Rotation2d.fromRadians(intakePivotSim.getAngleRads());
    inputs.angularVelocityRotsPerSec =
        RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.currentAmps = intakePivotSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.tempDegreesC = 0.0;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    intakePivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    setMotorVoltage(
        pivotPid.calculate(intakePivotSim.getAngleRads(), targetPosition.getRadians())
            + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
  }
}
