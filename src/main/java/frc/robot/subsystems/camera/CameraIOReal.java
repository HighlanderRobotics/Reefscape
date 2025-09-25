// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import frc.robot.subsystems.camera.Camera.CameraConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class CameraIOReal implements CameraIO {
  private final CameraConstants constants;

  public PhotonCamera camera;

  public CameraIOReal(CameraConstants constants) {
    this.constants = constants;
    this.camera = new PhotonCamera(constants.name());
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    var results = camera.getAllUnreadResults();

    if (results.size() > 0) {
      inputs.result = results.get(results.size() - 1);
      inputs.stale = false;
    } else {
      // else leave stale data
      inputs.stale = true;
    }
  }

  @Override
  public String getName() {
    return constants.name();
  }

  @Override
  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult) {}

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }
}
