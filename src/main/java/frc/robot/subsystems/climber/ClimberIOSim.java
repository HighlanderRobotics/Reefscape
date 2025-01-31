package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
  // this is stolen from the shoulder but its like fine our physics sim isnt really good enough for
  // full climb sim
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ClimberSubsystem.CLIMB_GEAR_RATIO,
          0.3,
          Units.inchesToMeters(13.5),
          0.0,
          Units.degreesToRadians(270.0),
          true,
          0.0);

  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(final ClimberIOInputsAutoLogged inputs) {
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
  public void setVoltage(final double voltage) {
    appliedVoltage = voltage;
    armSim.setInputVoltage(voltage);
  }

  @Override
  public void setPosition(final double position) {
    setVoltage(
        pid.calculate(
                armSim.getAngleRads(),
                Math.asin(
                    (Units.rotationsToRadians(position)
                            * ClimberSubsystem.CLIMBER_DRUM_RADIUS_METERS)
                        / ClimberSubsystem.CLIMBER_ARM_LENGTH_METERS))
            + feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }
}
