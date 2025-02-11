package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double angularVelocityRPS = 0.0;
    public Rotation2d position = new Rotation2d();
    public double tempDegreesC = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(ClimberIOInputsAutoLogged inputs);

  public void setVoltage(final double volts);

  public void setPosition(final double position);
}
