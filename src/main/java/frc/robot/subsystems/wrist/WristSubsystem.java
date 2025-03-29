package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.ExtensionKinematics;
import frc.robot.subsystems.shoulder.ShoulderIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  public static final double WRIST_GEAR_RATIO = 4.0 * 4.0 * (64.0 / 34.0);
  // TODO: UPDATE WHEN CAD IS FINISHED
  public static final Rotation2d MAX_ARM_ROTATION = Rotation2d.fromDegrees(220.0);
  public static final Rotation2d MIN_ARM_ROTATION = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d ZEROING_OFFSET = Rotation2d.fromDegrees(88.0);

  public static final Rotation2d WRIST_RETRACTED_POS = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d WRIST_HP_POS = Rotation2d.fromDegrees(160.0);
  public static final Rotation2d WRIST_CORAL_GROUND = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d WRIST_INTAKE_ALGAE_GROUND_POS = Rotation2d.fromDegrees(-50);
  public static final Rotation2d WRIST_INTAKE_ALGAE_STACK_POS = Rotation2d.fromDegrees(-50);
  public static final Rotation2d WRIST_SCORE_L1_POS = ExtensionKinematics.L1_EXTENSION.wristAngle();
  public static final Rotation2d WRIST_WHACK_L1_POS = Rotation2d.fromDegrees(-70);
  public static final Rotation2d WRIST_SCORE_L2_POS = ExtensionKinematics.L2_EXTENSION.wristAngle();
  public static final Rotation2d WRIST_SCORE_L3_POS = ExtensionKinematics.L3_EXTENSION.wristAngle();
  public static final Rotation2d WRIST_SCORE_L4_POS = ExtensionKinematics.L4_EXTENSION.wristAngle();
  public static final Rotation2d WRIST_CLEARANCE_POS = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d WRIST_INTAKE_ALGAE_REEF_POS = Rotation2d.fromDegrees(-20.0);
  public static final Rotation2d WRIST_INTAKE_ALGAE_REEF_RETRACT_POS =
      Rotation2d.fromDegrees(-20.0);
  public static final Rotation2d WRIST_SHOOT_NET_POS = Rotation2d.fromDegrees(70);
  public static final Rotation2d WRIST_SCORE_PROCESSOR_POS = WRIST_RETRACTED_POS;

  public static MotionMagicConfigs DEFAULT_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(4).withMotionMagicAcceleration(6);

  public static MotionMagicConfigs SLOW_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(4).withMotionMagicAcceleration(4);

  public static MotionMagicConfigs CRAWL_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(2);

  private final WristIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private Rotation2d setpoint = Rotation2d.kZero;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);

  public boolean hasZeroed = false;

  public WristSubsystem(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Wrist", inputs);
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Wrist/Has Zeroed", hasZeroed);
  }

  public Command setTargetAngle(final Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          setpoint = target.get();
          if (Robot.ROBOT_TYPE != RobotType.REAL)
            Logger.recordOutput("Carriage/Wrist/Setpoint", setpoint);
        });
  }

  public Command setTargetAngle(final Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Command setSlowTargetAngle(final Supplier<Rotation2d> target) {
    return this.runOnce(() -> io.setMotionMagicConfigs(SLOW_MOTION_MAGIC))
        .andThen(this.setTargetAngle(target))
        .finallyDo((interrupted) -> io.setMotionMagicConfigs(DEFAULT_MOTION_MAGIC));
  }

  public Command setSlowTargetAngle(final Rotation2d target) {
    return this.setSlowTargetAngle(() -> target);
  }

  public Command setCrawlTargetAngle(final Supplier<Rotation2d> target) {
    return this.runOnce(() -> io.setMotionMagicConfigs(CRAWL_MOTION_MAGIC))
        .andThen(this.setTargetAngle(target))
        .finallyDo((interrupted) -> io.setMotionMagicConfigs(DEFAULT_MOTION_MAGIC));
  }

  public Command setCrawlTargetAngle(final Rotation2d target) {
    return this.setCrawlTargetAngle(() -> target);
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

  public Command currentZero(Supplier<ShoulderIOInputsAutoLogged> shoulderInputs) {
    return Commands.sequence(
        this.runOnce(
            () -> {
              currentFilter.reset();
              System.out.println("Wrist Zeroing");
            }),
        this.run(() -> io.setMotorVoltage(-1.0))
            .until(() -> Math.abs(currentFilter.calculate(inputs.statorCurrentAmps)) > 10.0),
        this.runOnce(
            () -> {
              hasZeroed = true;
              io.resetEncoder(shoulderInputs.get().position.minus(ZEROING_OFFSET));
            }));
  }
}
