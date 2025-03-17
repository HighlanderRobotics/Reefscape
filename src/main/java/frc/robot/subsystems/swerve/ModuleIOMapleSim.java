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

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import frc.robot.utils.MaplePhoenixUtil;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Swerve/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOMapleSim implements ModuleIO {
  private final ModuleConstants constants;

  // Hardware
  // Must be accessible to sim subclass
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Signals
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveAppliedVolts;
  private final BaseStatusSignal driveCurrent;
  private final BaseStatusSignal driveSupplyCurrent;
  private final BaseStatusSignal driveTempC;

  private final BaseStatusSignal turnAbsolutePosition;
  private final BaseStatusSignal turnPosition;
  private final BaseStatusSignal turnVelocity;
  private final BaseStatusSignal turnAppliedVolts;
  private final BaseStatusSignal turnStatorCurrent;
  private final BaseStatusSignal turnSupplyCurrent;
  private final BaseStatusSignal turnTempC;

  // Control modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC driveControlVelocity =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final SwerveModuleSimulation simulation;

  public ModuleIOMapleSim(
      ModuleConstants moduleConstants,
      SwerveConstants swerveConstants,
      SwerveModuleSimulation simulation) {
    this.constants = moduleConstants;

    driveTalon = new TalonFX(moduleConstants.driveID(), "*");
    turnTalon = new TalonFX(moduleConstants.turnID(), "*");
    cancoder = new CANcoder(moduleConstants.cancoderID(), "*");

    driveTalon.getConfigurator().apply(swerveConstants.getDriveConfig());

    turnTalon.getConfigurator().apply(swerveConstants.getTurnConfig(moduleConstants.cancoderID()));

    cancoder
        .getConfigurator()
        .apply(swerveConstants.getCancoderConfig(moduleConstants.cancoderOffset()));

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTempC = driveTalon.getDeviceTemp();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnStatorCurrent = turnTalon.getStatorCurrent();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTempC = turnTalon.getDeviceTemp();

    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(
                driveTalon,
                Optional.of(moduleConstants),
                SignalType.DRIVE,
                ImmutableSet.of(drivePosition)),
            new Registration(
                turnTalon,
                Optional.of(moduleConstants),
                SignalType.STEER,
                ImmutableSet.of(turnPosition)));

    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        driveTempC,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent,
        turnTempC);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    this.simulation = simulation;
    simulation.useDriveMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerSim(driveTalon, true));
    simulation.useSteerMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            turnTalon,
            swerveConstants.getTurnMotorInverted(),
            cancoder,
            false,
            Angle.ofBaseUnits(moduleConstants.cancoderOffset().getRadians(), Radian)));
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        driveTempC,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent,
        turnTempC);

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTempC = driveTempC.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTempC = turnTempC.getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveSetpoint(
      final double metersPerSecond, final double forceNewtons, final boolean coastOut) {
    // Doesnt actually refresh drive velocity signal, but should be cached
    if (metersPerSecond == 0
        && forceNewtons == 0
        && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      setDriveVoltage(0.0);
    } else {
      driveTalon.setControl(
          driveControlVelocity
              .withVelocity(metersPerSecond)
              // meters -> motor rotations should be handled by SensorToMechanismRatio
              .withFeedForward(forceNewtons / (9.37 / 483.0)));
    }
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    turnTalon.setControl(turnPID.withPosition(rotation.getRotations()));
  }

  @Override
  public void setCurrent(double amps) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrent'");
  }

  @Override
  public void setCurrentLimits(CurrentLimitsConfigs configs) {
    driveTalon.getConfigurator().apply(configs);
  }
}
