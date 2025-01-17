package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {
  @AutoLog
  public static class BeambreakIOInputs {
    public boolean get = false;
  }

  public void updateInputs(BeambreakIOInputsAutoLogged inputs);
}
