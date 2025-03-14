// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Elevator IO using TalonFXs. */
public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX motor = new TalonFX(16, "*");
  private final TalonFX follower = new TalonFX(17, "*");

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torque = new TorqueCurrentFOC(0.0);
  private final MotionMagicExpoVoltage positionTorque =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true);

  // misusing type system here - these correspond to linear meters, NOT rotations
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();

  public ElevatorIOReal() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Carriage position meters in direction of elevator
    config.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO / (2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);

    // Carriage mass is 12lbs
    // Manipulator is 5lbs
    // First stage is ~4lbs
    // 16lbs counterspringing from stage 1 to carriage
    // (ie 16lbs of force pulling carriage up)
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 0.43832;
    // (483.0 / 9.37)
    //     * config.Feedback.SensorToMechanismRatio
    //     * Units.lbsToKilograms(12 + 5 + (4.0 / 2.0) - 16);
    config.Slot0.kS = 1.1062;
    config.Slot0.kV = 1.9542;
    // converts accel -> force, force -> motor torque, motor torque -> amperage
    config.Slot0.kA = 0.26245;
    // (483.0 / 9.37)$
    //     * config.Feedback.SensorToMechanismRatio
    //     * Units.lbsToKilograms(12 + 5 + (4.0 / 2.0));
    config.Slot0.kP = 69.925;
    config.Slot0.kD = 5.5908;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    // Fuck it we ball
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 8.0;
    // Estimated from slightly less than motor free speed
    config.MotionMagic.MotionMagicCruiseVelocity =
        (5500.0 / 60.0) / config.Feedback.SensorToMechanismRatio;

    config.MotionMagic.MotionMagicExpo_kV = 1.9542;
    config.MotionMagic.MotionMagicExpo_kA = 0.26245;

    motor.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(motor.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, voltage, statorCurrent, supplyCurrent, temp);
    motor.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent, temp);
    inputs.positionMeters = position.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setTarget(final double meters) {
    motor.setControl(positionTorque.withPosition(meters));
  }

  @Override
  public void setVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setCurrent(final double amps) {
    motor.setControl(torque.withOutput(amps));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
