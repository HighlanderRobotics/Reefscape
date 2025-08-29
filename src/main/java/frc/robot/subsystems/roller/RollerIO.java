// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double positionRotations = 0.0;
  }

  public void updateInputs(RollerIOInputsAutoLogged inputs);

  public void setVoltage(double voltage);

  public default void stop() {
    setVoltage(0.0);
  }

  public void setVelocity(double velocityRPS);

  public void setPosition(Rotation2d rot);

  public void resetEncoder(double position);
}
