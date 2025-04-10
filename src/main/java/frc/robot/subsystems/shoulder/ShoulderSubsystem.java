package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.ExtensionKinematics;
import frc.robot.utils.Tracer;
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
  public static final Rotation2d SHOULDER_HP_POS = Rotation2d.fromDegrees(53.0);
  public static final Rotation2d SHOULDER_CORAL_GROUND_POS = Rotation2d.fromDegrees(8.0);

  public static final Rotation2d SHOULDER_INTAKE_ALGAE_GROUND_POS =
      Rotation2d.fromRadians(0.505).plus(Rotation2d.fromDegrees(-1.0));
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_STACK_POS = Rotation2d.fromDegrees(30.0);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_REEF_POS = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d SHOULDER_INTAKE_ALGAE_REEF_RETRACT_POS =
      Rotation2d.fromDegrees(60.0);
  // may be incorrect as l2-3 poses are derived from ExtensionKinematics now
  public static final Rotation2d SHOULDER_SCORE_POS =
      ExtensionKinematics.L2_EXTENSION.shoulderAngle();
  public static final Rotation2d SHOULDER_WHACK_L1_POS = Rotation2d.fromDegrees(45);
  public static final Rotation2d SHOULDER_SCORE_L1_POS =
      ExtensionKinematics.L1_EXTENSION.shoulderAngle();
  public static final Rotation2d SHOULDER_SCORE_L4_POS =
      ExtensionKinematics.L4_EXTENSION.shoulderAngle();
  public static final Rotation2d SHOULDER_PRE_NET_POS = Rotation2d.fromDegrees(30);
  public static final Rotation2d SHOULDER_SHOOT_NET_POS = Rotation2d.fromDegrees(90);
  public static final Rotation2d SHOULDER_SCORE_PROCESSOR_POS = Rotation2d.fromDegrees(60.0);
  public static final Rotation2d SHOULDER_CLEARANCE_POS = Rotation2d.fromDegrees(80.0);
  public static final Rotation2d SHOULDER_TUCKED_CLEARANCE_POS = Rotation2d.fromDegrees(35.0);

  public static final MotionMagicConfigs DEFAULT_CONFIGS =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(1.0).withMotionMagicAcceleration(6.0);
  public static final MotionMagicConfigs TOSS_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(0.275)
          .withMotionMagicAcceleration(4.0);

  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();

  private Rotation2d setpoint = Rotation2d.kZero;

  private LoggedNetworkBoolean dashboardZero = new LoggedNetworkBoolean("Zero Shoulder");

  public ShoulderSubsystem(final ShoulderIO io) {
    this.io = io;
    io.updateInputs(inputs);
    rezero();
    SmartDashboard.putData(
        "Shoulder Zero", Commands.runOnce(() -> this.rezero()).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Shoulder", inputs);
    if (dashboardZero.get()) {
      Tracer.trace(
          "Shoulder/Zero",
          () -> {
            rezero();
            dashboardZero.set(false);
          });
    }
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Carriage/Shoulder/Cancoder Pos", getZeroingAngle());
  }

  public Rotation2d getZeroingAngle() {
    return Rotation2d.fromRotations(inputs.cancoderPosition).div(SHOULDER_FINAL_STAGE_RATIO);
  }

  public void rezero() {
    io.resetEncoder(getZeroingAngle());
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

  public Command setTargetAngleSlow(final Supplier<Rotation2d> target) {
    return Commands.sequence(
            this.runOnce(() -> io.setMotionMagicConfigs(TOSS_CONFIGS)),
            this.run(
                () -> {
                  io.setMotorPosition(target.get());
                  setpoint = target.get();
                  if (Robot.ROBOT_TYPE != RobotType.REAL)
                    Logger.recordOutput("Carriage/Shoulder/Setpoint", setpoint);
                }))
        .finallyDo(() -> io.setMotionMagicConfigs(DEFAULT_CONFIGS));
  }

  public Command setTargetAngleSlow(final Rotation2d target) {
    return setTargetAngleSlow(() -> target);
  }

  public Command setVoltage(final double volts) {
    return this.run(() -> io.setMotorVoltage(volts));
  }

  public Command hold() {
    return Commands.sequence(
        setTargetAngle(() -> inputs.position).until(() -> true), this.run(() -> {}));
  }

  public Rotation2d getAngle() {
    return inputs.position;
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public boolean isNearTarget() {
    return isNearAngle(setpoint);
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.position.getDegrees(), 10.0);
  }

  public boolean isNearAngle(Rotation2d target, Rotation2d tolerance) {
    return MathUtil.isNear(
        target.getDegrees(), inputs.position.getDegrees(), tolerance.getDegrees());
  }

  public ShoulderIOInputsAutoLogged getInputs() {
    return inputs;
  }

  public double getVelocity() {
    return inputs.angularVelocityRPS;
  }
}
