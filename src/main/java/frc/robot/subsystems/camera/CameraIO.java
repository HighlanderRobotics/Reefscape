// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.camera.Camera.CameraConstants;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public String name = "";
    // latency could just be calculated from the timestamp, do we need it as an input or could it be
    // an output?
    // public double latency = 0.0;
    // public PhotonTrackedTarget[] targets = new PhotonTrackedTarget[] {};
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public Transform3d coprocPNPTransform = new Transform3d();
    // public long sequenceID = 0;
    // public long captureTimestampMicros = 0;
    // public long publishTimestampMicros = 0;
    // public long timeSinceLastPong = 0;
    public boolean stale = true;
  }

  public void updateInputs(CameraIOInputs inputs);

  public void setSimPose(Optional<EstimatedRobotPose> simEst, Camera camera, boolean newResult);

  public String getName();

  public CameraConstants getCameraConstants();
}
