// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.utils.Tracer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorSubsystem extends RollerSubsystem {
  public static final String NAME = "Manipulator";

  public static final double CORAL_INTAKE_VELOCITY = -18.0;
  public static final double JOG_POS = 0.75;
  public static final double ALGAE_INTAKE_VOLTAGE = 10.0;
  public static final double ALGAE_HOLDING_VOLTAGE = 1.0;
  public static final double ALGAE_CURRENT_THRESHOLD = 6.0;
  public static final Transform2d IK_WRIST_TO_CORAL = ExtensionKinematics.IK_WRIST_TO_CORAL;

  public static final double CORAL_HOLD_POS = 0.5;

  public static final double GEAR_RATIO = (58.0 / 10.0) * (24.0 / 18.0);

  private final BeambreakIO firstBBIO, secondBBIO;
  private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged(),
      secondBBInputs = new BeambreakIOInputsAutoLogged();

  private boolean bb1 = false;
  private boolean bb2 = false;
  @AutoLogOutput private boolean hasAlgae = false;

  private LinearFilter currentFilter = LinearFilter.movingAverage(20);
  private double currentFilterValue = 0.0;

  private Timer zeroTimer = new Timer();

  private double positionSetpoint = 0.0;

  /** Creates a new Manipulator. */
  public ManipulatorSubsystem(RollerIO rollerIO, BeambreakIO firstBBIO, BeambreakIO secondBBIO) {
    super(rollerIO, NAME);
    this.firstBBIO = firstBBIO;
    this.secondBBIO = secondBBIO;
    zeroTimer.start();
  }

  @Override
  public void periodic() {
    // roller io is updated in superclass
    super.periodic();
    firstBBIO.updateInputs(firstBBInputs);
    secondBBIO.updateInputs(secondBBInputs);

    Logger.processInputs(NAME + "/First Beambreak", firstBBInputs);
    Logger.processInputs(NAME + "/Second Beambreak", secondBBInputs);
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput(NAME + "/Has Algae", hasAlgae);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim First Beambreak Override", bb1);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim Second Beambreak Override", bb2);

    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Filtered Current", currentFilterValue);

    if (getFirstBeambreak() && !getSecondBeambreak()) {
      Tracer.trace("Manipulator/Zero", () -> io.resetEncoder(0.0));
      zeroTimer.reset();
    }

    if (!getFirstBeambreak() && getSecondBeambreak()) {
      // Number calculated from coral length, may need tuning
      Tracer.trace("Manipulator/Zero", () -> io.resetEncoder(1.0));
      zeroTimer.reset();
    }
  }

  /** For the old ee */
  @Deprecated
  public Command index() {
    return Commands.sequence(
        setVelocity(9.0)
            .until(() -> getFirstBeambreak() || getSecondBeambreak())
            .unless(() -> getFirstBeambreak()),
        setVelocity(3.0).until(() -> getSecondBeambreak()).unless(() -> getSecondBeambreak()),
        setVelocity(-3.0)
            .until(() -> getFirstBeambreak() && !getSecondBeambreak())
            .unless(() -> zeroTimer.get() < 0.25),
        // TODO tune timeout
        // Commands.runOnce(() -> io.resetEncoder(0.0)),
        Commands.run(() -> io.setPosition(Rotation2d.fromRotations(1.1)))
            .until(() -> !getFirstBeambreak() && !getSecondBeambreak()));
  } // TODO check if anything got lost in merge?

  public Command jog(double rotations) {
    return Commands.sequence(
        // this.runOnce(() -> io.resetEncoder(0.0)),
        this.run(
            () -> {
              io.setPosition(Rotation2d.fromRotations(rotations));
              positionSetpoint = rotations;
            }));
  }

  public Command jog(DoubleSupplier rotations) {
    return Commands.sequence(
        // this.runOnce(() -> io.resetEncoder(0.0)),
        this.run(
            () -> {
              io.setPosition(Rotation2d.fromRotations(rotations.getAsDouble()));
              positionSetpoint = rotations.getAsDouble();
            }));
  }

  public Command hold() {
    return this.jog(() -> inputs.positionRotations)
        .until(() -> true)
        .andThen(this.run(() -> {}))
        .until(() -> !MathUtil.isNear(positionSetpoint, inputs.positionRotations, 2.0))
        .repeatedly();
  }

  public void resetPosition(final double rotations) {
    io.resetEncoder(rotations);
  }

  public Command intakeCoral() {
    return intakeCoral(CORAL_INTAKE_VELOCITY);
  }

  public Command intakeCoralAir(double vel) {
    return Commands.sequence(
        setVelocity(vel)
            .until(() -> getSecondBeambreak())
            .finallyDo(
                () -> {
                  io.setPosition(Rotation2d.fromRotations(0.63));
                  positionSetpoint = 0.63;
                }),
        setVoltage(2.0).until(() -> !getFirstBeambreak()),
        jog(CORAL_HOLD_POS).until(() -> !getSecondBeambreak() && !getFirstBeambreak()));
  }

  public Command intakeCoral(double vel) {
    return Commands.sequence(
        setVelocity(vel).until(new Trigger(() -> getSecondBeambreak()).debounce(0.5)),
        Commands.runOnce(
            () -> {
              io.setPosition(Rotation2d.fromRotations(0.5));
              positionSetpoint = 0.5;
            }),
        setVelocity(1.0).until(() -> !getFirstBeambreak()),
        jog(CORAL_HOLD_POS).until(() -> !getSecondBeambreak() && !getFirstBeambreak()));
  }

  public Command intakeAlgae() {
    return this.run(() -> io.setVoltage(ALGAE_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
                .debounce(0.75))
        .andThen(this.run(() -> io.setVoltage(ALGAE_HOLDING_VOLTAGE)));
  }

  public double getStatorCurrentAmps() {
    return currentFilterValue;
  }

  public double getTimeSinceZero() {
    return zeroTimer.get();
  }

  public boolean getFirstBeambreak() {
    return firstBBInputs.get || bb1;
  }

  public boolean getSecondBeambreak() {
    return secondBBInputs.get || bb2;
  }

  public void setFirstBeambreak(boolean state) {
    bb1 = state;
  }

  public void setSecondBeambreak(boolean state) {
    bb2 = state;
  }

  public void setHasAlgae(boolean state) {
    hasAlgae = state;
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }
}
