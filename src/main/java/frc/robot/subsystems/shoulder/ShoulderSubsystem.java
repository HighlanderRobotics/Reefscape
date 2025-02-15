package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ShoulderSubsystem extends SubsystemBase {
  // TODO: UPDATE WITH CAD
  public static final double SHOULDER_FINAL_STAGE_RATIO = 3.0;
  public static final double SHOULDER_GEAR_RATIO =
      25.0 * (34.0 / 28.0) * SHOULDER_FINAL_STAGE_RATIO;
  public static final int CANCODER_ID = 5;
  public static final Rotation2d MAX_SHOULDER_ROTATION = Rotation2d.fromDegrees(120.0);
  public static final Rotation2d MIN_SHOULDER_ROTATION = Rotation2d.fromDegrees(-5.0);
  public static final Rotation2d SHOULDER_RETRACTED_POS = Rotation2d.fromDegrees(90.0);

  public static final double X_OFFSET_METERS = 0.1016254;
  public static final double Z_OFFSET_METERS = 0.207645;
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.5);
  public static final Rotation2d SHOULDER_HP_POS = Rotation2d.fromDegrees(95.0);

  public static final Rotation2d SHOULDER_INTAKE_ALGAE_GROUND_POS = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_STACK_POS = Rotation2d.fromDegrees(30.0);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_REEF_POS = Rotation2d.fromDegrees(60.0);
  public static final Rotation2d SHOULDER_SCORE_POS = Rotation2d.fromDegrees(75);
  public static final Rotation2d SHOULDER_SCORE_L4_POS = Rotation2d.fromDegrees(45);
  public static final Rotation2d SHOULDER_SHOOT_NET_POS = Rotation2d.fromDegrees(90);
  public static final Rotation2d SHOULDER_SCORE_PROCESSOR_POS = SHOULDER_RETRACTED_POS;
  public static final Rotation2d SHOULDER_CLEARANCE_POS = Rotation2d.fromDegrees(80.0);

  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();

  private Rotation2d setpoint = Rotation2d.kZero;

  private LoggedNetworkBoolean dashboardZero = new LoggedNetworkBoolean("Zero Shoulder");

  public ShoulderSubsystem(final ShoulderIO io) {
    this.io = io;
    rezero();
    SmartDashboard.putData(
        "Shoulder Zero", Commands.runOnce(() -> this.rezero()).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Shoulder", inputs);
    if (dashboardZero.get()) {
      rezero();
      dashboardZero.set(false);
    }
  }

  public void rezero() {
    io.resetEncoder(
        inputs.cancoderPosition.div(SHOULDER_FINAL_STAGE_RATIO).plus(Rotation2d.kCCW_90deg));
  }

  public Command setTargetAngle(final Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          setpoint = target.get();
          Logger.recordOutput("Carriage/Shoulder/Setpoint", setpoint);
        });
  }

  public Command setTargetAngle(final Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Rotation2d getAngle() {
    return inputs.position;
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.position.getDegrees(), 10.0);
  }

  public ShoulderIOInputsAutoLogged getInputs() {
    return inputs;
  }
}
