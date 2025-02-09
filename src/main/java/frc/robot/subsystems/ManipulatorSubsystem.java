// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

public class ManipulatorSubsystem extends RollerSubsystem {
  public static final String NAME = "Manipulator";

  public static final double ALGAE_INTAKE_VOLTAGE = -10.0;
  public static final double ALGAE_HOLDING_VOLTAGE = -1.0;

  private final BeambreakIO firstBBIO, secondBBIO;
  private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged(),
      secondBBInputs = new BeambreakIOInputsAutoLogged();

  private boolean bb1 = false;
  private boolean bb2 = false;
  private boolean hasAlgae = false;

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
    Logger.recordOutput(NAME + "/Has Algae", hasAlgae);
  }

  public Command index() {
    return Commands.sequence(
        setVelocity(10.0).until(() -> firstBBInputs.get),
        setVelocity(3.0).until(() -> secondBBInputs.get),
        // TODO tune timeout
        setVelocity(1.0).withTimeout(0.5),
        setVelocity(0));
  }

  public Command backIndex() {
    return Commands.sequence(
        setVelocity(-INDEXING_VELOCITY).until(() -> !secondBBInputs.get), index());
  }

  public Command intakeAlgae() {
    return this.run(() -> io.setVoltage(ALGAE_INTAKE_VOLTAGE))
        .until(() -> inputs.statorCurrentAmps > 20.0)
        .andThen(this.run(() -> io.setVoltage(ALGAE_HOLDING_VOLTAGE)));
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
    return inputs.statorCurrentAmps;
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
