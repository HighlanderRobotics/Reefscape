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

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.Module.ModuleConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final ModuleConstants moduleConstants;
  private final SwerveConstants swerveConstants;

  private static final DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor steerMotor = DCMotor.getKrakenX60Foc(1);
  private final TalonFX driveTalon;
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC driveControlVelocity =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double turnAppliedVolts = 0.0;

  private final PIDController turnController = new PIDController(100.0, 0.0, 0.0);

  public ModuleIOSim(final ModuleConstants moduleConstants, final SwerveConstants swerveConstants) {
    this.moduleConstants = moduleConstants;
    this.swerveConstants = swerveConstants;
    driveTalon = new TalonFX(moduleConstants.driveID());
    driveTalon.getConfigurator().apply(swerveConstants.getDriveConfig());

    driveSim = // Third param is the moment of inertia of the swerve wheel
        // Used to approximate the robot inertia, not perfect but fine for the
        // Fidelity of simulation we are targeting
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotor, 0.025, swerveConstants.getDriveGearRatio()),
            driveMotor,
            0,
            0);

    turnSim = // Third param is the moment of inertia of the swerve steer
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                steerMotor, 0.0040, swerveConstants.getTurnGearRatio()),
            steerMotor,
            0,
            0);
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    final var driveSimState = driveTalon.getSimState();
    driveSimState.Orientation = ChassisReference.Clockwise_Positive;
    // driveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    driveSim.setInput(driveSimState.getMotorVoltage());

    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);
    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60.0) * swerveConstants.getDriveGearRatio());

    inputs.prefix = moduleConstants.prefix();

    inputs.drivePositionMeters =
        driveSim.getAngularPositionRad() * swerveConstants.getWheelRadiusMeters();
    inputs.driveVelocityMetersPerSec =
        driveSim.getAngularVelocityRadPerSec() * swerveConstants.getWheelRadiusMeters();
    inputs.driveAppliedVolts = driveSimState.getMotorVoltage();
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond, final double forceNewtons) {
    driveTalon.setControl(
        driveControlVelocity.withVelocity(metersPerSecond).withFeedForward(forceNewtons));
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    setTurnVoltage(
        turnController.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()));
  }
}
