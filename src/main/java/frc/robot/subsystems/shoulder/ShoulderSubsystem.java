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
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Tracer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ShoulderSubsystem extends SubsystemBase {
  public static final double SHOULDER_FINAL_STAGE_RATIO = 3.0;
  public static final double SHOULDER_GEAR_RATIO =
      25.0 * (34.0 / 28.0) * SHOULDER_FINAL_STAGE_RATIO;
  public static final int CANCODER_ID = 5;
  public static final Rotation2d MAX_SHOULDER_ROTATION = Rotation2d.fromDegrees(120.0);
  public static final Rotation2d MIN_SHOULDER_ROTATION = Rotation2d.fromDegrees(-5.0);

  public static final double X_OFFSET_METERS = 0.1016254;
  public static final double Z_OFFSET_METERS = 0.207645;
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.5);

  public static final MotionMagicConfigs DEFAULT_CONFIGS =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(1.0).withMotionMagicAcceleration(6.0);
  public static final MotionMagicConfigs TOSS_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(0.275)
          .withMotionMagicAcceleration(4.0);

  public enum ShoulderState {
    HP(50.0),
    PRE_INTAKE_CORAL_GROUND(35.0),
    INTAKE_CORAL_GROUND(8.0),
    PRE_L1(35.0),
    L1(Units.radiansToDegrees(1.617)), // not sure about units tbh
    PRE_L2(35.0),
    L2(Rotation2d.fromRadians(0.569).plus(Rotation2d.fromDegrees(20)).getDegrees()),
    PRE_L3(35.0),
    L3(Rotation2d.fromRadians(1.022).minus(Rotation2d.fromDegrees(3)).getDegrees()),
    PRE_L4(8.0),
    L4(25.0),
    PRE_INTAKE_ALGAE_REEF(35.0),
    INTAKE_ALGAE_REEF(45.0),
    INTAKE_ALGAE_STACK(30.0),
    INTAKE_ALGAE_GROUND(
        Rotation2d.fromRadians(0.505)
            .plus(Rotation2d.fromDegrees(-5.0))
            .minus(Rotation2d.fromDegrees(2))
            .getDegrees()), // hello??
    READY_ALGAE(60.0),
    PRE_BARGE(30),
    SCORE_BARGE(90),
    PROCESSOR(60.0),
    HOME(50.0);

    private final Supplier<Rotation2d> angle;

    private ShoulderState(double defaultAngle) {
      LoggedTunableNumber ltn = new LoggedTunableNumber("Shoulder/" + this.name(), defaultAngle);
      // we're in real life!! use degrees
      this.angle = () -> Rotation2d.fromDegrees(ltn.get());
    }

    public Rotation2d getAngle() {
      return angle.get();
    }
  }

  @AutoLogOutput(key = "Carriage/Shoulder/Setpoint")
  private Rotation2d setpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Carriage/Shoulder/State")
  private ShoulderState state = ShoulderState.HP;

  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();

  private LoggedNetworkBoolean dashboardZero = new LoggedNetworkBoolean("Zero Shoulder");

  public ShoulderSubsystem(ShoulderIO io) {
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
          "Carriage/Shoulder/Zero",
          () -> {
            rezero();
            dashboardZero.set(false);
          });
    }
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Carriage/Shoulder/Cancoder Pos", getZeroingAngle());
  }

  public void setState(ShoulderState state) {
    this.state = state;
  }

  public Command setStateAngle() {
    return setAngle(() -> state.getAngle());
  }

  @AutoLogOutput(key = "Carriage/Shoulder/Zeroing Angle")
  // what
  public Rotation2d getZeroingAngle() {
    return Rotation2d.fromRotations(inputs.cancoderPosition).div(SHOULDER_FINAL_STAGE_RATIO);
  }

  public void rezero() {
    io.setEncoderPosition(getZeroingAngle());
  }

  public Command setAngle(Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setPivotAngle(target.get());
          setpoint = target.get();
          Logger.recordOutput("Carriage/Shoulder/Setpoint", setpoint);
        });
  }

  public Command setAngle(Rotation2d target) {
    return setAngle(() -> target);
  }

  public Command setVoltage(double volts) {
    return this.run(() -> io.setMotorVoltage(volts));
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public ShoulderIOInputsAutoLogged getInputs() {
    return inputs;
  }

  public double getVelocity() {
    return inputs.angularVelocityRPS;
  }

  public Rotation2d getAngle() {
    return inputs.motorPosition;
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }
}
