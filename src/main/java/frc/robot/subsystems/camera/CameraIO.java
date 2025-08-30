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
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public boolean stale = true;
  }

  public void updateInputs(CameraIOInputs inputs);

  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult);

  public String getName();

  public CameraConstants getCameraConstants();
}
