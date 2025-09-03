package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ShoulderIOReal implements ShoulderIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<AngularVelocity> angularVelocityRPS;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);

  public ShoulderIOReal() {
    motor = new TalonFX(11, "*");
    cancoder = new CANcoder(ShoulderSubsystem.CANCODER_ID, "*");

    angularVelocityRPS = motor.getVelocity();
    temp = motor.getDeviceTemp();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    motorPositionRotations = motor.getPosition();
    appliedVoltage = motor.getMotorVoltage();
    cancoderPositionRotations = cancoder.getAbsolutePosition();

    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = 8.0;
    config.Slot0.kS = 3.5;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 8000.0;
    config.Slot0.kD = 160.0;

    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // guesses
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 4.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShoulderSubsystem.SHOULDER_GEAR_RATIO;

    // config.Feedback.RotorToSensorRatio =
    //     ShoulderSubsystem.SHOULDER_GEAR_RATIO / ShoulderSubsystem.SHOULDER_FINAL_STAGE_RATIO;
    // config.Feedback.SensorToMechanismRatio = ShoulderSubsystem.SHOULDER_FINAL_STAGE_RATIO;
    // config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset =
        -0.36; // -0.385174; // 0.2307 + 0.25; // 0.6323; // 0.779;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;

    cancoder.getConfigurator().apply(cancoderConfig);
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRPS,
        temp,
        appliedVoltage,
        supplyCurrentAmps,
        statorCurrentAmps,
        cancoderPositionRotations,
        motorPositionRotations);

    motor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    Logger.recordOutput(
        "Shoulder/Signal Refresh Status Code",
        BaseStatusSignal.refreshAll(
            angularVelocityRPS,
            temp,
            supplyCurrentAmps,
            statorCurrentAmps,
            motorPositionRotations,
            cancoderPositionRotations,
            appliedVoltage));

    inputs.motorPosition = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.cancoderPosition = cancoderPositionRotations.getValueAsDouble();
    inputs.tempDegreesC = temp.getValue().in(Units.Celsius);
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.angularVelocityRPS = angularVelocityRPS.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setPivotAngle(final Rotation2d targetPosition) {
    motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
  }

  @Override
  public void setEncoderPosition(final Rotation2d rotation) {
    motor.setPosition(rotation.getRotations());
  }

  @Override
  // TODO i hate this can we get rid of this
  public void setMotionMagicConfigs(final MotionMagicConfigs configs) {
    motor.getConfigurator().apply(configs);
  }
}
