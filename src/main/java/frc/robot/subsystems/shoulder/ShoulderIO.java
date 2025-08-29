package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  class ShoulderIOInputs {
    public double angularVelocityRPS = 0.0;
    public Rotation2d motorPosition = new Rotation2d();
    public double cancoderPosition = 0.0;
    public double appliedVoltage = 0.0;
    public double tempDegreesC = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(final ShoulderIOInputs inputs);

  public void setMotorVoltage(final double voltage);

  public void setPivotAngle(final Rotation2d targetPosition);

  public default void setEncoderPosition(final Rotation2d rotation) {}

  public default void resetEncoder() {
    setEncoderPosition(Rotation2d.kZero);
  }

  public void setMotionMagicConfigs(final MotionMagicConfigs configs);
}
