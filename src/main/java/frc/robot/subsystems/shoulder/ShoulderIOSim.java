package frc.robot.subsystems.shoulder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderIOSim implements ShoulderIO {
  // TODO: UPDATE WITH VALUES WHEN CAD IS DONE
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ShoulderSubsystem.SHOULDER_GEAR_RATIO,
          3.0,
          Units.inchesToMeters(13.5),
          ShoulderSubsystem.MIN_SHOULDER_ROTATION.getRadians(),
          ShoulderSubsystem.MAX_SHOULDER_ROTATION.getRadians(),
          true,
          0.0);

  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0); // 1.31085, 0.278);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(100.0, 0.0, 6.0, new TrapezoidProfile.Constraints(10.0, 10.0));

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(final ShoulderIOInputs inputs) {
    if (DriverStation.isDisabled()) armSim.setInput(0);
    armSim.update(0.02);

    inputs.angularVelocityRPS =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.statorCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.tempDegreesC = 0.0;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public void setMotorVoltage(final double voltage) {
    appliedVoltage = voltage;
    armSim.setInputVoltage(voltage);
  }

  @Override
  public void setMotorPosition(final Rotation2d targetPosition) {
    setMotorVoltage(
        pid.calculate(armSim.getAngleRads(), targetPosition.getRadians())
            + feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }

  @Override
  public void setMotionMagicConfigs(MotionMagicConfigs configs) {
    pid.setConstraints(
        new TrapezoidProfile.Constraints(
            configs.MotionMagicCruiseVelocity, configs.MotionMagicAcceleration));
  }
}
