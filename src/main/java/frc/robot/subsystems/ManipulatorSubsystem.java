// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

public class ManipulatorSubsystem extends RollerSubsystem {
  public static final String NAME = "Manipulator";

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final BeambreakIO firstBBIO, secondBBIO;
  private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged(), secondBBInputs = new BeambreakIOInputsAutoLogged();

  /** Creates a new Manipulator. */
  public ManipulatorSubsystem(RollerIO rollerIO, BeambreakIO firstBBIO, BeambreakIO secondBBIO) {
    super(rollerIO, NAME);
    this.rollerIO = rollerIO;
    this.firstBBIO = firstBBIO;
    this.secondBBIO = secondBBIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    firstBBIO.updateInputs(firstBBInputs);
    secondBBIO.updateInputs(secondBBInputs);

    Logger.processInputs(NAME + "/Roller", rollerInputs);
    Logger.processInputs(NAME + "/First Beambreak", firstBBInputs);
    Logger.processInputs(NAME + "/Second Beambreak", secondBBInputs);
  }

  public Command indexCmd() {
    return setVelocity(INDEXING_VELOCITY)
        .until(() -> !firstBBInputs.get && secondBBInputs.get)
        .andThen(setVelocity(0));
  }

  public boolean getFirstBeambreak() {
    return firstBBInputs.get;
  }

  public boolean getSecondBeambreak() {
    return secondBBInputs.get;
  }
}
