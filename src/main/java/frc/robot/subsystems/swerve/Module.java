// Copyright 2023-2025 FRC 8033
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** Wrapper around ModuleIO and ModuleIOInputs to organize module-level functionality. */
public class Module {
  // Represents per-module constants
  public record ModuleConstants(
      int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(final ModuleIO io) {
    this.io = io;
  }

  /** Update inputs without running the rest of the periodic logic. */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs(
        new StringBuilder("Swerve/").append(inputs.prefix).append(" Module").toString(), inputs);
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    return runSetpoint(state, 0.0, 0.0);
  }

  /** Runs the module closed loop with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(
      SwerveModuleState state, double forceXNewtons, double forceYNewtons) {
    // Optimize state based on current angle
    state.optimize(getAngle());
    // force on the motor is the total force vector projected onto the velocity vector
    // to project a onto b take ||a||*cos(theta) where theta is the angle between the two vectors
    // We want the magnitude of the projection, so we can ignore the direction of this later
    final var theta = Math.atan2(forceYNewtons, forceXNewtons) - inputs.turnPosition.getRadians();
    final double forceNewtons = Math.hypot(forceXNewtons, forceYNewtons) * Math.cos(theta);

    io.setTurnSetpoint(state.angle);
    io.setDriveSetpoint(
        state.speedMetersPerSecond * Math.cos(state.angle.minus(inputs.turnPosition).getRadians()),
        forceNewtons);
    Logger.recordOutput(
        new StringBuilder("Swerve/").append(inputs.prefix).append(" Force Feedforward").toString(),
        forceNewtons);
    return state;
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    state.optimize(getAngle());
    Logger.recordOutput(
        new StringBuilder("Swerve/").append(inputs.prefix).append(" Voltage Target").toString(),
        state.speedMetersPerSecond);

    io.setTurnSetpoint(state.angle);
    io.setDriveVoltage(
        state.speedMetersPerSecond * Math.cos(state.angle.minus(inputs.turnPosition).getRadians()),
        focEnabled);

    return state;
  }

  /**
   * Runs the module open loop with the specified setpoint state, velocity in volts. Returns the
   * optimized state. FOC is defaulted to on.
   */
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state) {
    return runVoltageSetpoint(state, true);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Returns the current turn angle of the module at normal sampling frequency. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters at normal sampling frequency. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns this modules prefix ie "Back Left" */
  public String getPrefix() {
    return inputs.prefix;
  }

  /**
   * Returns the current drive velocity of the module in meters per second withat normal sampling
   * frequency.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position) at normal sampling frequency. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity) at normal sampling frequency. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }
}
