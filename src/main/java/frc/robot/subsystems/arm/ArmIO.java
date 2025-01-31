package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double angularVelocityRPS = 0.0;
    public Rotation2d position = new Rotation2d();
    public double tempDegreesC = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  void updateInputs(final ArmIOInputs inputs);

  void setMotorVoltage(final double voltage);

  void setMotorPosition(final Rotation2d targetPosition);
}
