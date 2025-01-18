// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public interface VisionIO {

  public static class VisionIOInputs {
    public double timestamp = 0.0;
    // latency could just be calculated from the timestamp, do we need it as an input or could it be
    // an output?
    public double latency = 0.0;
    // We could use protobuf serialization for this instead of custom
    // There are som alleged performance considerations for protobuf
    // Could be worth testing? This works for now
    public List<PhotonTrackedTarget> targets = new ArrayList<>();
    public double numTags = 0; // Helps with deserialization
    public Transform3d coprocPNPTransform = new Transform3d();
    public VisionConstants constants =
        new VisionConstants(
            "Default",
            new Transform3d(),
            Matrix.eye(Nat.N3()),
            MatBuilder.fill(Nat.N8(), Nat.N1(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    public long sequenceID = 0;
    public long captureTimestampMicros = 0;
    public long publishTimestampMicros = 0;
    public long timeSinceLastPong = 0;
  }

  public void updateInputs(VisionIOInputs inputs);

  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult);

  public String getName();
}
