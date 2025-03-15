// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  // TODO update
  public static final double CLIMBER_ARM_LENGTH_METERS = Units.inchesToMeters(11.0);
  public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
  public static final double CLIMB_GEAR_RATIO = 25.0;
  public static final double CLIMB_EXTENDED_POSITION = 3.5;

  public static final Rotation2d UNLOCK_POSITION = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d LOCK_POSITION = Rotation2d.fromDegrees(0.0);

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final ServoIO servo;
  private final ServoIOInputsAutoLogged servoInputs = new ServoIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io, ServoIO servo) {
    this.io = io;
    this.servo = servo;

    SmartDashboard.putData(
        "rezero Climber", Commands.runOnce(() -> io.resetEncoder(0.0)).ignoringDisable(true));
    SmartDashboard.putData("Reset Climber (MANUAL STOP)", resetClimber());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // no servo inputs to update
    Logger.processInputs("Climber", inputs);
  }

  public Command setPosition(double position) {
    return this.run(() -> io.setPosition(position));
  }

  public Command resetClimber() {
    return this.run(() -> io.setVoltage(-8.0));
  }

  public Command zeroClimber() {
    return Commands.runOnce(() -> io.resetEncoder(0.0)).ignoringDisable(true);
  }

  public Command unlock() {
    return this.runOnce(() -> servo.setPosition(UNLOCK_POSITION));
  }

  public Command lock() {
    return this.runOnce(() -> servo.setPosition(LOCK_POSITION));
  }
}
