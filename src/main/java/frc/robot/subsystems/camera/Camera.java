// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;

/** Add your docs here. */
public class Camera {

  // TODO add doc comment about how the matrices are actually only used for sim cause i'd forgotten that!
  public record CameraConstants(
      String name,
      Transform3d robotToCamera,
      Matrix<N3, N3> intrinsicsMatrix,
      Matrix<N8, N1> distCoeffs) {}

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
  private final PhotonPoseEstimator estimator = new PhotonPoseEstimator(Robot.ROBOT_HARDWARE.swerveConstants.getFieldTagLayout(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);

  public Camera(CameraIO io) {
    this.io = io;
    estimator.setRobotToCameraTransform(io.getCameraConstants().robotToCamera);
    io.updateInputs(inputs);
  }

    public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void processApriltagInputs() {
    Logger.processInputs("Apriltag Vision/" + io.getName(), inputs);
  }

    public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    // Skip if we have no targets (could/should switch to 1?)
    if (result.getTargets().size() < 1) {
      return Optional.empty();
    }
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "Vision/" + io.getName() + " Best Distance",
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
    Optional<EstimatedRobotPose> estPose =
    estimator.update(result);
    return estPose;
  }

    public void setSimPose(Optional<EstimatedRobotPose> simEst, Camera camera, boolean newResult) {
    this.io.setSimPose(simEst, camera, newResult);
  }
}
