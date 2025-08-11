// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.camera.Camera.CameraConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class CameraIOSim implements CameraIO {

  private final CameraConstants constants;
  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim simCamera;

  public static Supplier<Pose3d> pose = () -> Pose3d.kZero; // TODO wtf

  public CameraIOSim(CameraConstants constants) {
    this.sim = new VisionSystemSim(constants.name());
    var cameraProp = new SimCameraProperties();
    // TODO Fix these constants
    cameraProp.setCalibration(1080, 960, constants.intrinsicsMatrix(), constants.distCoeffs());
    cameraProp.setCalibError(0.0, 0.0);
    cameraProp.setFPS(50.0);
    cameraProp.setAvgLatencyMs(30.0);
    cameraProp.setLatencyStdDevMs(5.0);
    this.camera = new PhotonCamera(constants.name());
    this.simCamera = new PhotonCameraSim(camera, cameraProp);
    simCamera.enableDrawWireframe(true);
    simCamera.setMaxSightRange(7.0);
    this.constants = constants;
    sim.addCamera(simCamera, constants.robotToCamera());

    try {
      final var field = Robot.ROBOT_HARDWARE.swerveConstants.getFieldTagLayout();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    sim.update(pose.get());
    var results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      inputs.result = results.get(results.size() - 1);
      // final var result = results.get(results.size() - 1);
      // inputs.latency = result.metadata.getLatencyMillis();

      // inputs.targets = new PhotonTrackedTarget[result.targets.size()];
      // for (int i = 0; i < result.targets.size(); i++) {
      //   inputs.targets[i] = result.targets.get(i);
      // }

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
  public void setSimPose(Optional<EstimatedRobotPose> simEst, Camera camera, boolean newResult) {
    simEst.ifPresentOrElse(
        est ->
            sim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult) sim.getDebugField().getObject("VisionEstimation").setPoses();
        });
  }

  @Override
  public String getName() {
    return constants.name();
  }

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }
}
