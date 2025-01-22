// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Cascading elevator */
public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  public static final double GEAR_RATIO = 4.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static final Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(90.0);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(51.8);

  public static final double L2_EXTENSION_METERS = Units.inchesToMeters(8.7);
  public static final double L3_EXTENSION_METERS = Units.inchesToMeters(26.0);
  public static final double L4_EXTENSION_METERS = Units.inchesToMeters(50.3);

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Carriage Pose", getCarriagePose());
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setTarget(meters.getAsDouble());
          Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-1.0);
              Logger.recordOutput("Elevator/Setpoint", Double.NaN);
            })
        .until(() -> inputs.statorCurrentAmps > 40.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

  public Pose3d getCarriagePose() {
    return new Pose3d(
        Units.inchesToMeters(4.5) + inputs.positionMeters * ELEVATOR_ANGLE.getCos(),
        0.0,
        Units.inchesToMeters(7.0) + inputs.positionMeters * ELEVATOR_ANGLE.getSin(),
        new Rotation3d());
  }

  public Pose3d getFirstStagePose() {
    return new Pose3d(
        Units.inchesToMeters(2.25)
            + (inputs.positionMeters / 2.0) * Math.cos(ELEVATOR_ANGLE.getRadians()),
        0.0,
        Units.inchesToMeters(4.25)
            + (inputs.positionMeters / 2.0) * Math.sin(ELEVATOR_ANGLE.getRadians()),
        new Rotation3d());
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }

  public boolean isNearExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.02);
  }
}
