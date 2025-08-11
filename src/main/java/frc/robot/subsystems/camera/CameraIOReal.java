// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import frc.robot.subsystems.camera.Camera.CameraConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

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
    inputs.name = constants.name();
    var results = camera.getAllUnreadResults();

    if (results.size() > 0) {
      inputs.result = results.get(results.size() - 1);
      // final var result = results.get(results.size() - 1);
      // inputs.latency = result.metadata.getLatencyMillis();

      // inputs.targets = new PhotonTrackedTarget[result.targets.size()];
      // for (int i = 0; i < result.targets.size(); i++) {
      //   inputs.targets[i] = result.targets.get(i);
      // }

      // inputs.coprocPNPTransform =
      //     result
      //         .multitagResult
      //         .map((pnpResult) -> pnpResult.estimatedPose.best)
      //         .orElse(Transform3d.kZero);
      // inputs.sequenceID = result.metadata.getSequenceID();
      // inputs.captureTimestampMicros = result.metadata.getCaptureTimestampMicros();
      // inputs.publishTimestampMicros = result.metadata.getPublishTimestampMicros();
      // inputs.timeSinceLastPong = result.metadata.timeSinceLastPong;
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
  public void setSimPose(Optional<EstimatedRobotPose> simEst, Camera camera, boolean newResult) {}

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }
}
