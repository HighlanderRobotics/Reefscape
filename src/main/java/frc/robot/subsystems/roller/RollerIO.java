// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

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
    public boolean firstBeambreak = false; //closer to elevator
    public boolean secondBeambreak = false; //closer to outtake
  }

  public void updateInputs(RollerIOInputsAutoLogged inputs);

  public void setVoltage(double voltage);

  public default void stop() {
    setVoltage(0.0);
  }

  public void setVelocity(double velocityRPS);

  /**
   * This method is meant to set a function to be called alongside updateInput to update a
   * simulation, such as for routing simulation
   */
  public void registerSimulationCallback(Consumer<RollerIOInputsAutoLogged> callback);
}
