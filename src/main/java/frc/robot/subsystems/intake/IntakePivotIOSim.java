package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
  // TODO: SET TO ACTUAL VALUES WHEN CAD IS FINISHED
  // Taken from Citrus circuits 2024 CAD
  private final SingleJointedArmSim intakePivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IntakePivotSubsystem.PIVOT_RATIO,
          0.07,
          Units.inchesToMeters(11.5),
          IntakePivotSubsystem.MIN_ANGLE.getRadians(),
          IntakePivotSubsystem.MAX_ANGLE.getRadians(),
          true,
          0.0);

  // TODO: TUNE
  private final ProfiledPIDController pivotPid =
      new ProfiledPIDController(5.0, 0.0, 2.6, new TrapezoidProfile.Constraints(10.0, 10.0));
  // TODO: TUNE
  private final ArmFeedforward pivotFf = new ArmFeedforward(0.0, 0.3856, 0.543); // 0.543

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    intakePivotSim.update(0.02);

    inputs.position = Rotation2d.fromRadians(intakePivotSim.getAngleRads());
    inputs.angularVelocityRotsPerSec =
        RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.statorCurrentAmps = intakePivotSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.tempDegreesC = 0.0;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    appliedVoltage = voltage;
    intakePivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    setMotorVoltage(
        pivotPid.calculate(intakePivotSim.getAngleRads(), targetPosition.getRadians())
            + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
  }

  public void setResetSimState() {
    intakePivotSim.setState(0.0, 0.0);
  }
}
