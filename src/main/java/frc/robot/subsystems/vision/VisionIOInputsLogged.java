package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * AutoLog does not work with PhotonTrackedTargets as of yet which is very sad but until it does
 * here is a manual implementation
 */
public class VisionIOInputsLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Latency", latency);

    for (int i = 0; i < targets.size(); i++) {
      VisionHelper.Logging.logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
    }
    table.put("NumTags", targets.size());
    table.put("Pose", coprocPNPTransform);
    table.put("Stale", stale);
    VisionHelper.Logging.logVisionConstants(constants, table);
    table.put("SequenceID", sequenceID);
    table.put("Capture Timestamp Micros", captureTimestampMicros);
    table.put("Publish Timestamp Micros", publishTimestampMicros);
    table.put("Time Since Last Pong", timeSinceLastPong);
  }

  @Override
  public void fromLog(LogTable table) {
    latency = table.get("Latency", latency);
    for (int i = 0; i < table.get("NumTags", numTags); i++) {
      this.targets.add(VisionHelper.Logging.getLoggedPhotonTrackedTarget(table, String.valueOf(i)));
    }
    numTags = table.get("NumTags", numTags);
    coprocPNPTransform = table.get("Pose", coprocPNPTransform);
    constants = VisionHelper.Logging.getLoggedVisionConstants(table);
    sequenceID = table.get("SequenceID", sequenceID);
    captureTimestampMicros = table.get("Capture Timestamp Micros", captureTimestampMicros);
    publishTimestampMicros = table.get("Publish Timestamp Micros", publishTimestampMicros);
    timeSinceLastPong = table.get("Time Since Last Pong", timeSinceLastPong);
    stale = table.get("Stale", stale);
  }

  public VisionIOInputsLogged clone() {
    VisionIOInputsLogged copy = new VisionIOInputsLogged();
    copy.latency = this.latency;
    copy.targets = this.targets;
    copy.numTags = this.numTags;
    copy.coprocPNPTransform = this.coprocPNPTransform;
    copy.sequenceID = this.sequenceID;
    copy.captureTimestampMicros = this.captureTimestampMicros;
    copy.publishTimestampMicros = this.publishTimestampMicros;
    copy.timeSinceLastPong = this.timeSinceLastPong;
    return copy;
  }
}
