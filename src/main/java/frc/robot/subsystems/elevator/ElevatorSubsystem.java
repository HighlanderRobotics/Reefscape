// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Cascading elevator */
public class ElevatorSubsystem extends SubsystemBase {
  // Constants
  public static final double GEAR_RATIO = 2.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static final Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(90.0);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(63.50);

  public static final double L1_EXTENSION_METERS = Units.inchesToMeters(12.0);
  public static final double L1_WHACK_CORAL_EXTENSION_METERS = Units.inchesToMeters(24.0);
  public static final double L2_EXTENSION_METERS = Units.inchesToMeters(16.0);
  public static final double L3_EXTENSION_METERS = Units.inchesToMeters(31.5);
  public static final double L4_EXTENSION_METERS = Units.inchesToMeters(58.0);

  public static final double INTAKE_ALGAE_GROUND_EXTENSION = Units.inchesToMeters(5.0);
  public static final double INTAKE_ALGAE_STACK_EXTENSION = Units.inchesToMeters(12.5);
  public static final double INTAKE_ALGAE_LOW_EXTENSION = Units.inchesToMeters(25.4);
  public static final double INTAKE_ALGAE_HIGH_EXTENSION = Units.inchesToMeters(40.5);

  public static final double ALGAE_NET_EXTENSION = Units.inchesToMeters(61.5);
  public static final double ALGAE_PROCESSOR_EXTENSION = 0.0;

  public static final double HP_EXTENSION_METERS = Units.inchesToMeters(1.0);

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  private double setpoint = 0.0;

  private final SysIdRoutine voltageSysid;
  private final SysIdRoutine currentSysid;

  // For dashboard
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(3.0, Units.feetToMeters(4.0));
  private final LoggedMechanismRoot2d
      root = // CAD distance from origin to center of carriage at full retraction
      mech2d.getRoot(
              "Elevator", (3.0 / 2.0) + Units.inchesToMeters(9.053), Units.inchesToMeters(12.689));
  private final LoggedMechanismLigament2d carriage =
      new LoggedMechanismLigament2d("Carriage", 0, ELEVATOR_ANGLE.getDegrees());

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    voltageSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdTestStateVolts", state.toString())),
            new Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    currentSysid =
        new SysIdRoutine(
            new Config(
                Volts.of(30.0).per(Second),
                Volts.of(120.0),
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdTestStateCurrent", state.toString())),
            new Mechanism((volts) -> io.setCurrent(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);

    carriage.setLength(inputs.positionMeters);
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Elevator/Mechanism2d", mech2d);

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Elevator/Carriage Pose", getCarriagePose());

    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Elevator/Has Zeroed", hasZeroed);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Elevator/Filtered Current", currentFilterValue);
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setTarget(meters.getAsDouble());
          setpoint = meters.getAsDouble();
          if (Robot.ROBOT_TYPE != RobotType.REAL)
            Logger.recordOutput("Elevator/Setpoint", setpoint);
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command hold() {
    return Commands.sequence(
        setExtension(() -> inputs.positionMeters).until(() -> true), this.run(() -> {}));
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-2.0);
              setpoint = 0.0;
              if (Robot.ROBOT_TYPE != RobotType.REAL)
                Logger.recordOutput("Elevator/Setpoint", Double.NaN);
            })
        .until(() -> Math.abs(currentFilterValue) > 50.0)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                io.resetEncoder(0.0);
                hasZeroed = true;
              }
            });
  }

  public Command runSysid() {
    final Function<SysIdRoutine, Command> runSysid =
        (routine) ->
            Commands.sequence(
                routine
                    .quasistatic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .quasistatic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)),
                Commands.waitUntil(() -> Math.abs(inputs.velocityMetersPerSec) < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)));
    return Commands.sequence(
        runCurrentZeroing(), runSysid.apply(voltageSysid), runSysid.apply(currentSysid));
  }

  public Command setVoltage(double voltage) {
    return this.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return this.setVoltage(voltage.getAsDouble());
  }

  public Command setCurrent(double amps) {
    return this.run(
        () -> {
          io.setCurrent(amps);
        });
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

  public double getSetpoint() {
    return setpoint;
  }

  public boolean isNearExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public boolean isNearExtension(double expected, double toleranceMeters) {
    return MathUtil.isNear(expected, inputs.positionMeters, toleranceMeters);
  }
}
