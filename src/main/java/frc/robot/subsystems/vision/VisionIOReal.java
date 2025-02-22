// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.vision.Vision.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  // constants
  public String cameraName;
  public PhotonCamera camera;
  public Matrix<N3, N3> cameraMatrix;
  public Matrix<N8, N1> distCoeffs;
  private final VisionConstants constants;

  /*** Transform3d from the center of the robot to the camera mount position (ie,
   *     robot ➔ camera) in the <a href=
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>.
   ***/
  public Transform3d robotToCamera;

  public VisionIOReal(VisionConstants constants) {
    cameraName = constants.cameraName();
    camera = new PhotonCamera(cameraName);
    robotToCamera = constants.robotToCamera();
    cameraMatrix = constants.intrinsicsMatrix();
    distCoeffs = constants.distCoeffs();
    this.constants = constants;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      final var result = results.get(results.size() - 1);
      inputs.latency = result.metadata.getLatencyMillis();
      inputs.targets = result.targets;
      inputs.constants = constants;
      inputs.coprocPNPTransform =
          result
              .multitagResult
              .map((pnpResult) -> pnpResult.estimatedPose.best)
              .orElse(Transform3d.kZero);
      inputs.sequenceID = result.metadata.getSequenceID();
      inputs.captureTimestampMicros = result.metadata.getCaptureTimestampMicros();
      inputs.publishTimestampMicros = result.metadata.getPublishTimestampMicros();
      inputs.timeSinceLastPong = result.metadata.timeSinceLastPong;
      inputs.stale = false;
    } else {
      // else leave stale data
      inputs.stale = true;
    }
  }

  @Override
  public String getName() {
    return cameraName;
  }

  @Override
  public void setSimPose(Optional<EstimatedRobotPose> simEst, Vision camera, boolean newResult) {}
}
