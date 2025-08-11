package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOInputsLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Latency", latency);

    for (int i = 0; i < targets.size(); i++) {
      table.put(String.valueOf(i), targets.get(i));
    }
    table.put("NumTags", targets.size());
    table.put("Pose", coprocPNPTransform);
    table.put("Stale", stale);
    table.put("SequenceID", sequenceID);
    table.put("Capture Timestamp Micros", captureTimestampMicros);
    table.put("Publish Timestamp Micros", publishTimestampMicros);
    table.put("Time Since Last Pong", timeSinceLastPong);
  }

  @Override
  public void fromLog(LogTable table) {
    latency = table.get("Latency", latency);
    for (int i = 0; i < table.get("NumTags", numTags); i++) {
      this.targets.add(table.get(String.valueOf(i), new PhotonTrackedTarget())); // TODO uhhh
    }
    numTags = table.get("NumTags", numTags);
    coprocPNPTransform = table.get("Pose", coprocPNPTransform);
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
