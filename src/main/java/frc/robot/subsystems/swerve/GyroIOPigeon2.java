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

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import java.util.Optional;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2(int id) {
    pigeon = new Pigeon2(id, "*");
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(PhoenixOdometryThread.ODOMETRY_FREQUENCY_HZ);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(pigeon, Optional.empty(), SignalType.GYRO, ImmutableSet.of(yaw)));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = new Rotation2d(yaw.getValue());
    inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }
}
