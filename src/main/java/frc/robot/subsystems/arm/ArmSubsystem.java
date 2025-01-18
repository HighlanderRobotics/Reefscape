package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {
  // TODO: UPDATE WITH CAD
  public static final double ARM_GEAR_RATIO = 1.0;

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
