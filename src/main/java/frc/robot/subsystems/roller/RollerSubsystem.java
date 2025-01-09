// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.genericRoller.RollerIOInputsAutoLogged;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  /** Creates a new RollerSubsystem. */
  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
