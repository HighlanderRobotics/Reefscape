// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import java.util.function.Consumer;

public class RollerIOReal implements RollerIO {
  private final TalonFX motor;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
  private final PositionVoltage positionVoltage =
      new PositionVoltage(0.0).withEnableFOC(true).withSlot(1);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Angle> position;

  private Optional<Consumer<RollerIOInputsAutoLogged>> callback = Optional.empty();

  public RollerIOReal(final int motorID, final TalonFXConfiguration config) {
    this.motor = new TalonFX(motorID, "*");

    motor.getConfigurator().apply(config);

    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();
    position = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, voltage, statorCurrent, supplyCurrent, temp, position);

    motor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getDefaultConfig() {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(15.0)
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(new Slot0Configs().withKV(0.12).withKP(0.01))
        .withSlot1(new Slot1Configs().withKP(20))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage, statorCurrent, supplyCurrent, temp, position);
    inputs.velocityRotationsPerSec = velocity.getValue().in(RotationsPerSecond);
    inputs.appliedVolts = voltage.getValue().in(Volts);
    inputs.statorCurrentAmps = statorCurrent.getValue().in(Amps);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
    inputs.positionRotations = position.getValueAsDouble();

    callback.ifPresent((cb) -> cb.accept(inputs));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setVelocity(double velocityRPS) {
    motor.setControl(velocityVoltage.withVelocity(velocityRPS));
  }

  @Override
  public void registerSimulationCallback(Consumer<RollerIOInputsAutoLogged> callback) {
    this.callback = Optional.of(callback);
  }

  @Override
  public void setPosition(Rotation2d rot) {
    motor.setControl(positionVoltage.withPosition(rot.getRotations()));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
