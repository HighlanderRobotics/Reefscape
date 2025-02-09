package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class WristIOReal implements WristIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> angularVelocityRPS;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Angle> motorPositionRotations;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public WristIOReal(final int motorId, final TalonFXConfiguration config) {
    motor = new TalonFX(motorId, "*");

    angularVelocityRPS = motor.getVelocity();
    temp = motor.getDeviceTemp();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    motorPositionRotations = motor.getPosition();
    appliedVoltage = motor.getMotorVoltage();

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();

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
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRPS,
        temp,
        supplyCurrentAmps,
        statorCurrentAmps,
        motorPositionRotations,
        appliedVoltage);

    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
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

  public static TalonFXConfiguration getDefaultConfiguration() {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20.0)
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine))
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }
}
