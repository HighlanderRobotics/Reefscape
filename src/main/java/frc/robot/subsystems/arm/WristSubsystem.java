package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  public static final double WRIST_GEAR_RATIO = 1.0;
  // TODO: UPDATE WHEN CAD IS FINISHED
  public static final Rotation2d MAX_ARM_ROTATION = Rotation2d.fromDegrees(180.0);
  public static final Rotation2d MIN_ARM_ROTATION = Rotation2d.fromDegrees(-90.0);

  public static final Rotation2d WRIST_RETRACTED_POS = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d WRIST_HP_POS = Rotation2d.fromDegrees(256.5);
  public static final Rotation2d WRIST_INTAKE_ALGAE_GROUND_POS = Rotation2d.fromDegrees(164.9);
  public static final Rotation2d WRIST_SCORE_L1_POS = Rotation2d.fromDegrees(100.0 - 180 + 45);
  public static final Rotation2d WRIST_SCORE_L2_POS = Rotation2d.fromDegrees(140.0 - 180 + 45);
  public static final Rotation2d WRIST_SCORE_L3_POS = Rotation2d.fromDegrees(140.0 - 180 + 45);
  public static final Rotation2d WRIST_SCORE_L4_POS = Rotation2d.fromDegrees(125.0 - 180 + 45);
  public static final Rotation2d WRIST_SHOOT_NET_POS = Rotation2d.fromDegrees(215.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public WristSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Wrist", inputs);
  }

  public Command setTargetAngle(final Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          Logger.recordOutput("Carriage/Wrist/Setpoint", target.get());
        });
  }

  public Command setTargetAngle(final Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Rotation2d getAngle() {
    return inputs.position;
  }
}
