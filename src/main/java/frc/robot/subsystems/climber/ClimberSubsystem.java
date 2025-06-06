// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // TODO update
  public static final double CLIMBER_ARM_LENGTH_METERS = Units.inchesToMeters(11.0);
  public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
  public static final double CLIMB_GEAR_RATIO = 4.0 * 4.0 * 5.0;
  public static final double CLIMB_EXTENDED_POSITION = 3.4;

  public static final double FAST_VEL = 2.0;
  public static final double SLOW_VEL = 0.5;

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;

    SmartDashboard.putData(
        "rezero Climber", Commands.runOnce(() -> io.resetEncoder(0.0)).ignoringDisable(true));
    SmartDashboard.putData("Reset Climber (MANUAL STOP)", resetClimber());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setPosition(double position) {
    return this.run(() -> io.setPosition(position, FAST_VEL));
  }

  public Command setPositionSlow(double position) {
    return this.run(() -> io.setPosition(position, SLOW_VEL));
  }

  public Command resetClimber() {
    return this.run(() -> io.setVoltage(-8.0));
  }

  public Command zeroClimber() {
    return Commands.runOnce(() -> io.resetEncoder(0.0)).ignoringDisable(true);
  }
}
