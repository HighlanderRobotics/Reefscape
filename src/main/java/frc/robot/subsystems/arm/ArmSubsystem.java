package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  // TODO: UPDATE WITH CAD
  public static final double ARM_GEAR_RATIO = 1.0;
  public static final Rotation2d MAX_ARM_ROTATION = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ARM_ROTATION = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d ARM_RETRACTED_POS = Rotation2d.fromDegrees(0.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command setTargetAngle(Supplier<Rotation2d> target) {
    return this.runOnce(() -> Logger.recordOutput("Arm/Setpoint", target.get()))
        .andThen(this.run(() -> io.setMotorPosition(target.get())));
  }

  public Command setTargetAngle(Rotation2d target) {
    return setTargetAngle(() -> target);
  }
}
