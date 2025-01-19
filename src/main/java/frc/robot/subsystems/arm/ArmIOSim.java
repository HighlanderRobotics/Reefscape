package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  // TODO: UPDATE WITH VALUES WHEN CAD IS DONE
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ArmSubsystem.ARM_GEAR_RATIO,
          0.05,
          Units.inchesToMeters(26.04),
          0.0,
          Units.degreesToRadians(180),
          true,
          0.0);

  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);

    inputs.angularVelocityRPS =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.statorCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.tempDegreesC = 0.0;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    armSim.setInputVoltage(voltage);
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    setMotorVoltage(
        pid.calculate(armSim.getAngleRads(), targetPosition.getRadians())
            + feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }
}
