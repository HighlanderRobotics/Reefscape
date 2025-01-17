package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
  private IntakePivotIO io;
  private IntakePivotIOInputsAutoLogged inputs;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);
  }
}
