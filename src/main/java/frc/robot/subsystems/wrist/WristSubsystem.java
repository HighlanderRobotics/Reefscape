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
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  public static final double WRIST_GEAR_RATIO = 4.0 * 4.0 * (64.0 / 34.0);
  public static final Rotation2d MAX_ARM_ROTATION = Rotation2d.fromDegrees(220.0);
  public static final Rotation2d MIN_ARM_ROTATION = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d ZEROING_OFFSET = Rotation2d.fromRadians(1.451);
  public static final Rotation2d WRIST_RETRACTED_POS = Rotation2d.fromDegrees(20.0);

  // public static final Rotation2d WRIST_READY_ALGAE = Rotation2d.fromDegrees(-10.0);
  // public static final Rotation2d WRIST_HP_POS = Rotation2d.fromDegrees(178.0);
  // public static final Rotation2d WRIST_CORAL_GROUND = Rotation2d.fromDegrees(0.0);
  // public static final Rotation2d WRIST_INTAKE_ALGAE_GROUND_POS = Rotation2d.fromDegrees(-65);
  // public static final Rotation2d WRIST_INTAKE_ALGAE_STACK_POS = Rotation2d.fromDegrees(-10);
  // public static final Rotation2d WRIST_SCORE_L1_POS =
  // ExtensionKinematics.L1_EXTENSION.wristAngle();
  // public static final Rotation2d WRIST_WHACK_L1_POS = Rotation2d.fromDegrees(-70);
  // public static final Rotation2d WRIST_SCORE_L2_POS =
  // ExtensionKinematics.L2_EXTENSION.wristAngle();
  // public static final Rotation2d WRIST_SCORE_L3_POS =
  // ExtensionKinematics.L3_EXTENSION.wristAngle();
  // public static final Rotation2d WRIST_SCORE_L4_POS =
  // ExtensionKinematics.L4_EXTENSION.wristAngle();
  // public static final Rotation2d WRIST_CLEARANCE_POS = Rotation2d.fromDegrees(30.0);
  // public static final Rotation2d WRIST_TUCKED_CLEARANCE_POS = Rotation2d.fromDegrees(170.0);
  // public static final Rotation2d WRIST_INTAKE_ALGAE_REEF_POS = Rotation2d.fromDegrees(-20.0);
  // public static final Rotation2d WRIST_INTAKE_ALGAE_REEF_RETRACT_POS =
  //     Rotation2d.fromDegrees(-20.0);
  // public static final Rotation2d WRIST_SHOOT_NET_POS = Rotation2d.fromDegrees(110);
  // public static final Rotation2d WRIST_PRE_NET_POS = Rotation2d.fromDegrees(100);
  // public static final Rotation2d WRIST_SCORE_PROCESSOR_POS = Rotation2d.fromDegrees(-30.0);

  public static MotionMagicConfigs DEFAULT_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(5);

  public static MotionMagicConfigs SLOW_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(3);

  public static MotionMagicConfigs CRAWL_MOTION_MAGIC =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(2);

  public enum WristState {
    PRE_INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(30.0)), // formerly WRIST_CLEARANCE_POS
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(0.0)),
    HP(Rotation2d.fromDegrees(178.0)),
    PRE_L1(Rotation2d.fromRadians(0.349)),
    PRE_L2(Rotation2d.fromDegrees(170.0)),
    L2(Rotation2d.fromRadians(2.447)),
    PRE_L3(Rotation2d.fromDegrees(170.0)),
    L3(Rotation2d.fromRadians(2.427)),
    L4(Rotation2d.fromDegrees(120.0)), // ??
    PRE_INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(30.0)),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(-20.0)),
    INTAKE_ALGAE_STACK(Rotation2d.fromDegrees(-10)),
    INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(-65)),
    READY_ALGAE(Rotation2d.fromDegrees(20)),
    PRE_BARGE(Rotation2d.fromDegrees(100)),
    SCORE_BARGE(Rotation2d.fromDegrees(110)),
    PROCESSOR(Rotation2d.fromDegrees(-30.0)),
    HOME(Rotation2d.fromRadians(-0.687 - 1.0)) // i dunno
  ;

    private final Rotation2d angle;

    private WristState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  @AutoLogOutput(key = "Carriage/Wrist/Setpoint")
  private Rotation2d setpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Carriage/Wrist/State")
  private WristState state = WristState.HP;

  private final WristIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  public double currentFilterValue = 0.0;

  @AutoLogOutput(key = "Carriage/Wrist/Has Zeroed")
  public boolean hasZeroed = false;

  // i hate myself
  private Supplier<Rotation2d> shoulderAngleSupplier;

  public WristSubsystem(WristIO io, Supplier<Rotation2d> shoulderAngleSupplier) {
    this.io = io;
    this.shoulderAngleSupplier = shoulderAngleSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Wrist", inputs);
    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Carriage/Wrist/Filtered Current", currentFilterValue);
  }

  public void setState(WristState state) {
    this.state = state;
  }

  public Command setStateAngle() {
    return setAngle(() -> state.getAngle());
  }

  public Command setAngle(final Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setAngle(target.get());
          setpoint = target.get();
          Logger.recordOutput("Carriage/Wrist/Setpoint", setpoint);
        });
  }

  public Command setVoltage(final double volts) {
    return this.run(() -> io.setMotorVoltage(volts));
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

  public boolean atSetpoint() {
    return isNearAngle(setpoint);
  }

  public Command currentZero() {
    // return Commands.sequence(
    //         this.runOnce(
    //             () -> {
    //               currentFilter.reset();
    //               System.out.println("Wrist Zeroing");
    //             }),
    //         this.run(() -> io.setMotorVoltage(-1.0))
    //             .raceWith(
    //                 Commands.waitSeconds(0.5)
    //                     .andThen(
    //                         Commands.waitUntil(
    //                             () ->
    //                                 Math.abs(currentFilter.calculate(inputs.statorCurrentAmps))
    //                                     > 7.0))),
    //         this.runOnce(
    //             () -> {
    //               // Logger.recordOutput(
    //               //     "shoulder zero pos",
    // shoulderInputs.get().position.minus(ZEROING_OFFSET));
    //               hasZeroed = true;
    //               // io.resetEncoder(shoulderInputs.get().position.minus(ZEROING_OFFSET));
    //               io.resetEncoder(Rotation2d.fromRadians(-0.687));
    //             }))
    //     .finallyDo(() -> Commands.print("DONE"));
    return Commands.print("Wrist Zeroing")
        .andThen(
            this.run(() -> io.setMotorVoltage(-1.0))
                .until(() -> Math.abs(currentFilterValue) > 7.0)
                .finallyDo(
                    (interrupted) -> {
                      if (!interrupted) {
                        io.resetEncoder(Rotation2d.fromRadians(-0.687));
                        hasZeroed = true;
                      }
                    }));

    // return Commands.print("Elevator Zeroing")
    // .andThen(
    //     this.run(
    //             () -> {
    //               io.setVoltage(-2.0);
    //               setpoint = 0.0;
    //               if (Robot.ROBOT_TYPE != RobotType.REAL)
    //                 Logger.recordOutput("Elevator/Setpoint", Double.NaN);
    //             })
    //         .until(() -> Math.abs(currentFilterValue) > 50.0)
    //         .finallyDo(
    //             (interrupted) -> {
    //               if (!interrupted) {
    //                 io.resetEncoder(0.0);
    //                 hasZeroed = true;
    //               }
    //             }));
  }

  public void resetPosition(Rotation2d angle) {
    io.resetEncoder(angle);
    hasZeroed = true;
  }
}
