package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double angularVelocityRPS = 0.0;
    public Rotation2d position = new Rotation2d();
    public double appliedVoltage = 0.0;
    public double tempDegreesC = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(final ArmIOInputs inputs);

  public void setMotorVoltage(final double voltage);

  public void setMotorPosition(final Rotation2d targetPosition);

  public default void resetEncoder(final Rotation2d rotation) {}

  public default void resetEncoder() {
    resetEncoder(Rotation2d.kZero);
  }
}
