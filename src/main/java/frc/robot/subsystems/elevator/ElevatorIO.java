// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public void updateInputs(final ElevatorIOInputsAutoLogged inputs);

  public void setTarget(final double meters);

  public void setVoltage(final double voltage);

  public void setCurrent(final double amps);

  public default void stop() {
    setVoltage(0);
  }

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
