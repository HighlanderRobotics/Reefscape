package frc.robot.subsystems.servo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;

public class ServoIOReal implements ServoIO {
  private final Servo servo;

  public ServoIOReal(int id) {
    servo = new Servo(id);
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    servo.setAngle(rotation.getDegrees());
  }
}
