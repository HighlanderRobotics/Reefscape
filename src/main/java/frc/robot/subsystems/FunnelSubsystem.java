package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;

public class FunnelSubsystem extends SubsystemBase {
  private final RollerIO leftIO, rightIO;
  private final RollerIOInputsAutoLogged leftInputs = new RollerIOInputsAutoLogged(), rightInputs = new RollerIOInputsAutoLogged();

  public FunnelSubsystem(RollerIO leftIO, RollerIO rightIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;
  }

  @Override
  public void periodic() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);
    Logger.processInputs("Funnel/Left", leftInputs);
    Logger.processInputs("Funnel/Right", rightInputs);
  }

  public Command runVoltage(final double volts) {
    return this.run(() -> {
      leftIO.setVoltage(volts);
      rightIO.setVoltage(volts);
    });
  }
}
