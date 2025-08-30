// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
  public static final double X_OFFSET_METERS = Units.inchesToMeters(4.0);
  /** Offset from origin to center of pivot */
  public static final double Z_OFFSET_METERS = Units.inchesToMeters(8.175000);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(63.50);

  public static final double MAX_ACCELERATION = 10.0;
  public static final double SLOW_ACCELERATION = 5.0;
  public static final double MEDIUM_ACCELERATION = 8.5;

  public enum ElevatorState {
    HP(Units.inchesToMeters(0.0)),
    INTAKE_CORAL_GROUND(Units.inchesToMeters(0.0)),
    L1(0.217),
    L2(0.23 + Units.inchesToMeters(1.5)),
    L3(0.60 + Units.inchesToMeters(2.0)),
    L4(1.383),
    INTAKE_ALGAE_LOW(Units.inchesToMeters(20.0)),
    INTAKE_ALGAE_HIGH(Units.inchesToMeters(35.0)),
    INTAKE_ALGAE_STACK(Units.inchesToMeters(9.0)),
    INTAKE_ALGAE_GROUND(0.14 - Units.inchesToMeters(0.75)),
    READY_ALGAE(0.1),
    BARGE(Units.inchesToMeters(62.5)),
    PROCESSOR(Units.inchesToMeters(0.01)), // lmao
    HOME(-0.3), // i'm quite scared
    ANTIJAM_ALGAE(0.0) // NOT ACTUALLY 0!!!
  ;

    private final DoubleSupplier extensionMeters;

    private ElevatorState(double defaultExtensionMeters) {
      // this.extensionMeters = extensionMeters;
      this.extensionMeters =
          new LoggedTunableNumber("Elevator/" + this.name(), defaultExtensionMeters);
    }

    public double getExtensionMeters() {
      return extensionMeters.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Elevator/State")
  public ElevatorState state = ElevatorState.HP;

  private ElevatorState prevState = ElevatorState.HP;

  @AutoLogOutput(key = "Elevator/Setpoint")
  private double setpoint = 0.0;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  @AutoLogOutput(key = "Elevator/Has Zeroed")
  public static boolean hasZeroed = false;

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
    // if (Robot.ROBOT_TYPE != RobotType.REAL)
    Logger.recordOutput("Elevator/Filtered Current", currentFilterValue);
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setPosition(meters.getAsDouble(), MAX_ACCELERATION);
          setpoint = meters.getAsDouble();
          Logger.recordOutput("Elevator/Setpoint", setpoint);
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command setStateExtension() {
    return setExtension(() -> state.getExtensionMeters());
  }

  public boolean atExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public boolean atExtension() {
    return atExtension(setpoint);
  }

  public Command runCurrentZeroing() {
    return Commands.print("Elevator Zeroing")
        .andThen(
            this.run(
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
                    }));
  }

  public Pose3d getCarriagePose() {
    return new Pose3d(
        Units.inchesToMeters(4.5) + inputs.positionMeters * ELEVATOR_ANGLE.getCos(),
        0.0,
        Units.inchesToMeters(7.0) + inputs.positionMeters * ELEVATOR_ANGLE.getSin(),
        new Rotation3d());
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void resetExtension(double extension) {
    System.out.println("Elevator zeroing");
    io.resetEncoder(extension);
    hasZeroed = true;
    System.out.println("Elevator zeroed!");
  }
}
