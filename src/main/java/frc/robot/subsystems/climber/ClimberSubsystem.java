// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // TODO update
  public static final double CLIMBER_ARM_LENGTH_METERS = Units.inchesToMeters(11.0);
  public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
  public static final double CLIMB_GEAR_RATIO = 64.0;
  public static final double CLIMB_EXTENDED_POSITION = 10.0;

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setPosition(double position) {
    return this.run(() -> io.setPosition(position));
  }
}
