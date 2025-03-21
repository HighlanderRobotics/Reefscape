// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.Tracer;
import org.littletonrobotics.junction.Logger;

public class ManipulatorSubsystem extends RollerSubsystem {
  public static final String NAME = "Manipulator";

  public static final double ALGAE_INTAKE_VOLTAGE = -10.0;
  public static final double ALGAE_HOLDING_VOLTAGE = -3.0;
  public static final double ALGAE_CURRENT_THRESHOLD = 30.0;

  private final BeambreakIO firstBBIO, secondBBIO;
  private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged(),
      secondBBInputs = new BeambreakIOInputsAutoLogged();

  private boolean bb1 = false;
  private boolean bb2 = false;
  private boolean hasAlgae = false;

  private LinearFilter currentFilter = LinearFilter.movingAverage(20);
  private double currentFilterValue = 0.0;

  /** Creates a new Manipulator. */
  public ManipulatorSubsystem(RollerIO rollerIO, BeambreakIO firstBBIO, BeambreakIO secondBBIO) {
    super(rollerIO, NAME);
    this.firstBBIO = firstBBIO;
    this.secondBBIO = secondBBIO;
  }

  @Override
  public void periodic() {
    // roller io is updated in superclass
    super.periodic();
    firstBBIO.updateInputs(firstBBInputs);
    secondBBIO.updateInputs(secondBBInputs);

    Logger.processInputs(NAME + "/First Beambreak", firstBBInputs);
    Logger.processInputs(NAME + "/Second Beambreak", secondBBInputs);
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Superstructure" + "/Has Algae", hasAlgae);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim First Beambreak Override", bb1);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Sim Second Beambreak Override", bb2);

    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(NAME + "/Filtered Current", currentFilterValue);

    if (firstBBInputs.get && !secondBBInputs.get) {
      Tracer.trace("Manipulator/Zero", () -> io.resetEncoder(0.0));
    }
  }

  public Command index() {
    return Commands.sequence(
        setVelocity(9.0).until(() -> firstBBInputs.get).unless(() -> firstBBInputs.get),
        setVelocity(3.0).until(() -> secondBBInputs.get).unless(() -> secondBBInputs.get),
        // TODO tune timeout
        // Commands.runOnce(() -> io.resetEncoder(0.0)),
        Commands.run(() -> io.setPosition(Rotation2d.fromRotations(1.1))),
        // setVelocity(2.0).withTimeout(0.25),
        setVelocity(0));
  } // TODO check if anything got lost in merge?

  public Command jog(double rotations) {
    return Commands.sequence(
        // this.runOnce(() -> io.resetEncoder(0.0)),
        this.run(() -> io.setPosition(Rotation2d.fromRotations(rotations))));
  }

  public Command hold() {
    return this.jog(inputs.positionRotations).until(() -> true).andThen(this.run(() -> {}));
  }

  public Command backIndex() {
    return Commands.sequence(
        setVelocity(-INDEXING_VELOCITY).until(() -> !secondBBInputs.get), index());
  }

  public Command intakeAlgae() {
    return this.run(() -> io.setVoltage(ALGAE_INTAKE_VOLTAGE))
        .until(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
        .andThen(
            this.runOnce(() -> hasAlgae = true),
            this.run(() -> io.setVoltage(ALGAE_HOLDING_VOLTAGE)));
  }

  public Pose3d getPose(
      ShoulderSubsystem shoulder,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      Pose2d robotPose) {
    return new Pose3d( // Manipulator
        new Translation3d(
            ShoulderSubsystem.X_OFFSET_METERS
                + shoulder.getAngle().getCos() * ShoulderSubsystem.ARM_LENGTH_METERS
                + robotPose.getX(),
            robotPose.getY(),
            elevator.getExtensionMeters()
                + ShoulderSubsystem.Z_OFFSET_METERS
                + shoulder.getAngle().getSin() * ShoulderSubsystem.ARM_LENGTH_METERS),
        new Rotation3d(0, wrist.getAngle().getRadians(), Math.PI));
  }

  public double getStatorCurrentAmps() {
    return currentFilterValue;
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
