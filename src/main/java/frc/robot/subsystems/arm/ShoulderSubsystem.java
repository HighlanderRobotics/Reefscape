package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShoulderSubsystem extends SubsystemBase {
  // TODO: UPDATE WITH CAD
  public static final double SHOULDER_FINAL_STAGE_RATIO = 3.0;
  public static final double SHOULDER_GEAR_RATIO = 9.0 * (34.0 / 28.0) * SHOULDER_FINAL_STAGE_RATIO;
  public static final int CANCODER_ID = 5;
  public static final Rotation2d MAX_SHOULDER_ROTATION = Rotation2d.fromDegrees(120.0);
  public static final Rotation2d MIN_SHOULDER_ROTATION = Rotation2d.fromDegrees(-5.0);
  public static final Rotation2d SHOULDER_RETRACTED_POS = Rotation2d.fromDegrees(80.0);

  public static final double X_OFFSET_METERS = 0.1016254;
  public static final double Z_OFFSET_METERS = 0.207645;
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.5);
  public static final Rotation2d SHOULDER_HP_POS = Rotation2d.fromDegrees(104.95);

  public static final Rotation2d SHOULDER_INTAKE_ALGAE_GROUND_POS = Rotation2d.fromDegrees(26.0);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_STACK_POS = Rotation2d.fromDegrees(35);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_REEF_POS = Rotation2d.fromDegrees(68.5);
  public static final Rotation2d SHOULDER_SCORE_POS = Rotation2d.fromDegrees(75);
  public static final Rotation2d SHOULDER_SHOOT_NET_POS = Rotation2d.fromDegrees(90);
  // TODO: SET TO CORRECT POS
  public static final Rotation2d SHOULDER_SCORE_PROCESSOR_POS = Rotation2d.fromDegrees(0.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ShoulderSubsystem(final ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Shoulder", inputs);
  }

  public Command setTargetAngle(final Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          Logger.recordOutput("Carriage/Shoulder/Setpoint", target.get());
        });
  }

  public Command setTargetAngle(final Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Rotation2d getAngle() {
    return inputs.position;
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.position.getDegrees(), 10.0);
  }
}
