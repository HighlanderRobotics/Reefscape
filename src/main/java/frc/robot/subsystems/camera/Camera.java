// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.Tracer;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Camera {

  // TODO add doc comment about how the matrices are actually only used for sim cause i'd forgotten
  // that!
  public record CameraConstants(
      String name,
      Transform3d robotToCamera,
      Matrix<N3, N3> intrinsicsMatrix,
      Matrix<N8, N1> distCoeffs) {}

  public static final Matrix<N3, N1> visionPointBlankDevs =
      new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.6, 0.6, 0.5});
  public static final Matrix<N3, N1> infiniteDevs =
      new Matrix<N3, N1>(
          Nat.N3(),
          Nat.N1(),
          new double[] {
            Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
          });
  public static final double distanceFactor = 3.0;

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
  private final PhotonPoseEstimator estimator =
      new PhotonPoseEstimator(
          Robot.ROBOT_HARDWARE.swerveConstants.getFieldTagLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          null);

  private Alert futureVisionData;

  private Pose3d pose;

  public Camera(CameraIO io) {
    this.io = io;
    estimator.setRobotToCameraTransform(io.getCameraConstants().robotToCamera);
    io.updateInputs(inputs);
    futureVisionData =
        new Alert(
            getName() + " Vision Data Coming from ✨The Future✨",
            AlertType.kError); // what the hell is this
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
    Optional<EstimatedRobotPose> estPose = estimator.update(result);
    return estPose;
  }

  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult) {
    this.io.setSimPose(simEst, newResult);
  }

  public String getName() {
    return io.getName();
  }

  public static Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    var deviation = visionPointBlankDevs.times(Math.max(avgDistance, 0.0) * distanceFactor);
    if (estimation.targetsUsed.size() == 1) {
      deviation = deviation.times(3);
    }
    if (estimation.targetsUsed.size() == 1 && estimation.targetsUsed.get(0).poseAmbiguity > 0.15) {
      return infiniteDevs;
    }
    // Reject if estimated pose is in the air or ground
    if (Math.abs(estimation.estimatedPose.getZ()) > 0.125) {
      return infiniteDevs;
    }
    // TAG_COUNT_DEVIATION_PARAMS
    //     .get(
    //         MathUtil.clamp(
    //             estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
    //     .computeDeviation(avgDistance);
    return deviation;
  }

  public void updateCamera() {
    var hasFutureData = false;
    try {
      if (!inputs.stale) {
        var estPose = Tracer.trace("Update Camera", () -> update(inputs.result));
        var visionPose = estPose.get().estimatedPose;
        pose = visionPose;
        // Sets the pose on the sim field
        setSimPose(estPose, !inputs.stale);

        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/Vision Pose From " + getName(), visionPose);
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/Vision Pose2d From " + getName(), visionPose.toPose2d());
        final var deviations = findVisionMeasurementStdDevs(estPose.get());
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Deviations", deviations.getData());

        Tracer.trace(
            "Add Measurement From " + getName(),
            () -> {
              SwerveSubsystem
                  .addVisionMeasurement( // TODO this makes me uncomfortable but i don't really know
                      // if it's an issue
                      visionPose.toPose2d(),
                      inputs.result.metadata.captureTimestampMicros / 1.0e6,
                      deviations
                          // .times(DriverStation.isAutonomous() ? 2.0 : 1.0)
                          .times(
                              getName().equals("Front_Left_Camera")
                                      || getName().equals("Front_Right_Camera")
                                  ? 0.75
                                  : 2.0)
                          // reef positions
                          .times(
                              (getName().equals("Front_Left_Camera")
                                          || getName().equals("Front_Right_Camera"))
                                      && (Robot.state.get().toString().startsWith("PRE_L")
                                          || Robot.state.get() == SuperState.SCORE_CORAL
                                          || Robot.state.get() == SuperState.INTAKE_ALGAE_HIGH
                                          || Robot.state.get() == SuperState.INTAKE_ALGAE_LOW)
                                  ? 0.5
                                  : 1.5) // TODO tune these sorts of numbers
                          // hp tags
                          .times(
                              // !camera.getName().equals("Front_Camera")
                              // &&
                              estPose.get().targetsUsed.stream()
                                      .anyMatch(
                                          t ->
                                              t.getFiducialId() == 12
                                                  || t.getFiducialId() == 13
                                                  || t.getFiducialId() == 2
                                                  || t.getFiducialId() == 1)
                                  ? 1.5
                                  : 1.0)
                          // barge tags
                          .times(
                              // !camera.getName().equals("Front_Right_Camera")
                              // &&
                              estPose.get().targetsUsed.stream()
                                      .anyMatch(
                                          t ->
                                              t.getFiducialId() == 4
                                                  || t.getFiducialId() == 5
                                                  || t.getFiducialId() == 15
                                                  || t.getFiducialId() == 14)
                                  ? 1.2
                                  : 1.0)
                          .times(Robot.state.get() == SuperState.PRE_NET ? 0.5 : 1.0));
              // the sussifier
            });

        hasFutureData |= inputs.result.metadata.captureTimestampMicros > RobotController.getTime();
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Good Update");
        
        Tracer.trace(
            "Log Tag Poses",
            () -> {
              Pose3d[] targetPose3ds = new Pose3d[inputs.result.targets.size()];
              for (int j = 0; j < inputs.result.targets.size(); j++) {
                targetPose3ds[j] =
                    Robot.ROBOT_HARDWARE
                        .swerveConstants
                        .getFieldTagLayout()
                        .getTagPose(inputs.result.targets.get(j).getFiducialId())
                        .get();
              }
              // if (Robot.ROBOT_TYPE != RobotType.REAL)
              Logger.recordOutput("Vision/" + getName() + "/Target Poses", targetPose3ds);
            });

      } else {
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Stale");
      }
    } catch (NoSuchElementException e) {
      // if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Bad Estimate");
    }
    futureVisionData.set(hasFutureData);
  }

  public CameraConstants getCameraConstants() {
    return io.getCameraConstants();
  }

  public Pose3d getPose() {
    return pose;
  }

  public boolean hasFrontTags() {
    return getName().equals("Front_Left_Camera")
        || getName().equals("Front_Right_Camera") && inputs.result.targets.size() > 0;
  }
}
