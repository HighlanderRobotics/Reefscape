package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public double tempDegreesC = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setMotorVoltage(double voltage);

  void setMotorPosition(Rotation2d targetPosition);
}
