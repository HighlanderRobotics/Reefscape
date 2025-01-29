package frc.robot.subsystems.servo;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {
  // Blank bc no sensors
  @AutoLog
  public static class ServoIOInputs {}

  public void setPosition(final Rotation2d rotation);
}
