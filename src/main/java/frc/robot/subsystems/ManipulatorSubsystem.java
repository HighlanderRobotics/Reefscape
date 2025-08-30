// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorSubsystem extends RollerSubsystem {
  public static final String NAME = "Manipulator";

  public static final double MAX_VELOCITY = 20; // holy cooked

  public static final double CORAL_INTAKE_VELOCITY = -18.0;
  public static final double JOG_POS = 0.75;
  public static final double ALGAE_INTAKE_VOLTAGE = 10.0;
  public static final double ALGAE_HOLDING_VOLTAGE = 1.0;
  public static final double ALGAE_CURRENT_THRESHOLD = 6.0;

  public static final double CORAL_HOLD_POS = 0.6;

  public static final double GEAR_RATIO = (58.0 / 10.0) * (24.0 / 18.0);

  private final BeambreakIO firstBBIO, secondBBIO;
  private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged(),
      secondBBInputs = new BeambreakIOInputsAutoLogged();

  private boolean bb1Sim = false;
  private boolean bb2Sim = false;
  @AutoLogOutput private boolean hasAlgaeSim = false;

  @AutoLogOutput(key = "Manipulator State Velocity")
  private double stateVelocity = 0.0;

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
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput(NAME + "/Has Algae", hasAlgaeSim);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim First Beambreak Override", bb1Sim);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim Second Beambreak Override", bb2Sim);

    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Filtered Current", currentFilterValue);

    if (getFirstBeambreak() && !getSecondBeambreak()) {
      zeroTimer.reset();
    }

    if (!getFirstBeambreak() && getSecondBeambreak()) {
      zeroTimer.reset();
    }
  }

  public void resetPosition(final double rotations) {
    io.resetEncoder(rotations);
  }

  public Command intakeAlgae() {
    return this.run(() -> io.setVoltage(ALGAE_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
                .debounce(0.75))
        .andThen(this.run(() -> io.setVoltage(ALGAE_HOLDING_VOLTAGE)));
  }

  public Command setStateVelocity(BooleanSupplier checkExtension) {
    return Commands.waitUntil(checkExtension).andThen(setRollerVelocity(stateVelocity));
  }

  public void setState(double vel) {
    stateVelocity = vel;
  }

  public double getStatorCurrentAmps() {
    return currentFilterValue;
  }

  public boolean hasAlgae() { // TODO icky
    return getStatorCurrentAmps() > ALGAE_CURRENT_THRESHOLD || hasAlgaeSim;
  }

  public boolean getFirstBeambreak() {
    return firstBBInputs.get || bb1Sim;
  }

  public boolean getSecondBeambreak() {
    return secondBBInputs.get || bb2Sim;
  }

  public boolean eitherBeambreak() {
    return getFirstBeambreak() || getSecondBeambreak();
  }

  public boolean bothBeambreaks() {
    return getFirstBeambreak() && getSecondBeambreak();
  }

  public boolean neitherBeambreak() {
    return !eitherBeambreak();
  }

  public void setSimFirstBeambreak(boolean b) {
    bb1Sim = b;
  }

  public void setSimSecondBeambreak(boolean b) {
    bb2Sim = b;
  }

  public void setSimHasAlgae(boolean state) {
    hasAlgaeSim = state;
  }

  public double getTimeSinceZero() {
    return zeroTimer.get();
  }
}
