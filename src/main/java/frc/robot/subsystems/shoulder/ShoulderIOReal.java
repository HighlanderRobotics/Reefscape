package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

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
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

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
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    // guesses
    config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
    config.MotionMagic.MotionMagicAcceleration = 10.0;

    config.Feedback.SensorToMechanismRatio = ShoulderSubsystem.SHOULDER_FINAL_STAGE_RATIO;

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;

    cancoder.getConfigurator().apply(cancoderConfig);
    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRPS,
        temp,
        appliedVoltage,
        supplyCurrentAmps,
        statorCurrentAmps,
        motorPositionRotations);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRPS,
        temp,
        supplyCurrentAmps,
        statorCurrentAmps,
        motorPositionRotations,
        cancoderPositionRotations,
        appliedVoltage);

    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.cancoderPosition =
        Rotation2d.fromRotations(cancoderPositionRotations.getValueAsDouble());
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
  public void setMotorPosition(final Rotation2d targetPosition) {
    motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
  }

  @Override
  public void resetEncoder(final Rotation2d rotation) {
    motor.setPosition(rotation.getRotations());
  }
}
