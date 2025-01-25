package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.InvertedDigitalInput;

public class BeambreakIOReal implements BeambreakIO {
  final DigitalInput beambreak;

  public BeambreakIOReal(int id, boolean invert) {
    beambreak = invert ? new InvertedDigitalInput(id) : new DigitalInput(id);
  }

  public void updateInputs(BeambreakIOInputsAutoLogged inputs) {
    inputs.get = beambreak.get();
  }
}
