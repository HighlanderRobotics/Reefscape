// Copyright 2023-2025 FRC 8033
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.swerve.OdometryThreadIO.OdometryThreadIOInputs;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionHelper;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.utils.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveConstants constants;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR
  private final OdometryThreadIO odoThread;
  private final OdometryThreadIOInputs odoThreadInputs = new OdometryThreadIOInputs();

  private SwerveDriveKinematics kinematics;

  private final Vision[] cameras;

  /** For delta tracking with PhoenixOdometryThread* */
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d rawGyroRotation = new Rotation2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private SwerveDrivePoseEstimator estimator;
  private double lastEstTimestamp = 0.0;
  private double lastOdometryUpdateTimestamp = 0.0;
  final Pose3d[] cameraPoses;

  private final Optional<SwerveDriveSimulation> simulation;

  private Alert usingSyncOdometryAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);

  private boolean useModuleForceFF = !Robot.isSimulation();

  public SwerveSubsystem(
      SwerveConstants constants,
      GyroIO gyroIO,
      VisionIO[] visionIOs,
      ModuleIO[] moduleIOs,
      OdometryThreadIO odoThread,
      Optional<SwerveDriveSimulation> simulation) {
    this.constants = constants;
    this.kinematics = new SwerveDriveKinematics(constants.getModuleTranslations());
    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.6, 0.6, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.4));
    this.gyroIO = gyroIO;
    this.odoThread = odoThread;
    this.simulation = simulation;
    cameras = new Vision[visionIOs.length];
    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = new Vision(visionIOs[i]);
    }

    cameraPoses = new Pose3d[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      cameraPoses[i] = Pose3d.kZero;
    }
  }

  public void startOdoThread() {
    // I don't love this but it seems to work
    odoThread.start();
  }

  public void periodic() {
    Tracer.trace(
        "SwervePeriodic",
        () -> {
          for (var camera : cameras) {
            Tracer.trace("Update cam inputs", camera::updateInputs);
            Tracer.trace("Process cam inputs", camera::processInputs);
          }
          Tracer.trace(
              "Update odo inputs",
              () -> odoThread.updateInputs(odoThreadInputs, lastOdometryUpdateTimestamp));
          Logger.processInputs("Async Odo", odoThreadInputs);
          if (!odoThreadInputs.sampledStates.isEmpty()) {
            lastOdometryUpdateTimestamp =
                odoThreadInputs
                    .sampledStates
                    .get(odoThreadInputs.sampledStates.size() - 1)
                    .timestamp();
          }
          Tracer.trace("update gyro inputs", () -> gyroIO.updateInputs(gyroInputs));
          for (int i = 0; i < modules.length; i++) {
            Tracer.trace(
                "SwerveModule update inputs from " + modules[i].getPrefix() + " Module",
                modules[i]::updateInputs);
          }
          Logger.processInputs("Swerve/Gyro", gyroInputs);
          for (int i = 0; i < modules.length; i++) {
            Tracer.trace(
                "SwerveModule periodic from " + modules[i].getPrefix() + " Module",
                modules[i]::periodic);
          }

          // Stop moving when disabled
          if (DriverStation.isDisabled()) {
            for (var module : modules) {
              module.stop();
            }
          }
          // Log empty setpoint states when disabled
          if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
          }

          Tracer.trace("Update odometry", this::updateOdometry);
          Tracer.trace("Update vision", this::updateVision);
        });
  }

  private void updateOdometry() {
    Logger.recordOutput("Swerve/Updates Since Last", odoThreadInputs.sampledStates.size());
    var sampleStates = odoThreadInputs.sampledStates;
    if (sampleStates.size() == 0
        || sampleStates.get(0).values().isEmpty()
        // We don't simulate async odo rn
        || Robot.ROBOT_TYPE == RobotType.SIM) {
      usingSyncOdometryAlert.set(true);
      sampleStates = getSyncSamples();
    } else {
      usingSyncOdometryAlert.set(false);
    }

    for (var sample : sampleStates) {
      // Read wheel deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas =
          new SwerveModulePosition[4]; // change in positions since the last update
      boolean hasNullModulePosition = false;
      // Technically we could have not 4 modules worth of data here but if we have a design that
      // goof we can deal later
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        var dist = sample.values().get(new SignalID(SignalType.DRIVE, moduleIndex));
        if (dist == null) {
          // No value at this timestamp
          hasNullModulePosition = true;

          Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, false);
          break;
        }

        var rot = sample.values().get(new SignalID(SignalType.STEER, moduleIndex));
        if (rot == null) {
          // No value at this timestamp
          hasNullModulePosition = true;

          Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, false);
          break;
        }

        // all our data is good!
        modulePositions[moduleIndex] =
            new SwerveModulePosition(
                dist, Rotation2d.fromRotations(rot)); // gets positions from the thread, NOT inputs

        Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, true);
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // missing some data :(
      if (hasNullModulePosition) {
        missingModuleData.set(true);
        if (!gyroInputs.isConnected
            || sample.values().get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID))
                == null) {
          missingGyroData.set(true);
          // no modules and no gyro so we're just sad :(
        } else {
          missingGyroData.set(false);
          // null here is checked by if clause
          rawGyroRotation =
              Rotation2d.fromDegrees(sample.values().get(new SignalID(SignalType.GYRO, -1)));
          lastGyroRotation = rawGyroRotation;
          Logger.recordOutput("Odometry/Gyro Rotation", lastGyroRotation);
          Tracer.trace(
              "update estimator",
              () ->
                  estimator.updateWithTime(
                      sample.timestamp(), rawGyroRotation, lastModulePositions));
        }
        continue;
      }

      // If we have all our module data . . .
      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      if (!gyroInputs.isConnected
          || sample.values().get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID))
              == null) {
        missingGyroData.set(true);
        // We don't have a complete set of data, so just use the module rotations
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      } else {
        missingGyroData.set(false);
        rawGyroRotation =
            Rotation2d.fromDegrees(
                sample
                    .values()
                    .get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID)));
        twist =
            new Twist2d(twist.dx, twist.dy, rawGyroRotation.minus(lastGyroRotation).getRadians());
      }
      // Apply the twist (change since last sample) to the current pose
      lastGyroRotation = rawGyroRotation;
      Logger.recordOutput("Odometry/Gyro Rotation", lastGyroRotation);
      // Apply update
      estimator.updateWithTime(sample.timestamp(), rawGyroRotation, modulePositions);
    }
  }

  private void updateVision() {
    var i = 0;
    for (var camera : cameras) {
      final PhotonPipelineResult result =
          new PhotonPipelineResult(
              camera.inputs.sequenceID,
              camera.inputs.captureTimestampMicros,
              camera.inputs.publishTimestampMicros,
              camera.inputs.timeSinceLastPong,
              camera.inputs.targets);
      try {
        if (!camera.inputs.stale) {
          var estPose = camera.update(result);
          var visionPose = estPose.get().estimatedPose;
          // Sets the pose on the sim field
          camera.setSimPose(estPose, camera, !camera.inputs.stale);
          Logger.recordOutput("Vision/Vision Pose From " + camera.getName(), visionPose);
          Logger.recordOutput(
              "Vision/Vision Pose2d From " + camera.getName(), visionPose.toPose2d());
          final var deviations = VisionHelper.findVisionMeasurementStdDevs(estPose.get());
          Logger.recordOutput("Vision/" + camera.getName() + "/Deviations", deviations.getData());
          estimator.addVisionMeasurement(
              visionPose.toPose2d(), camera.inputs.captureTimestampMicros / 1.0e6, deviations);
          lastEstTimestamp = camera.inputs.captureTimestampMicros / 1e6;
          Logger.recordOutput("Vision/" + camera.getName() + "/Invalid Pose Result", "Good Update");
          cameraPoses[i] = visionPose;
          Pose3d[] targetPose3ds = new Pose3d[result.targets.size()];
          for (int j = 0; j < result.targets.size(); j++) {
            targetPose3ds[j] =
                Robot.ROBOT_HARDWARE
                    .swerveConstants
                    .getFieldTagLayout()
                    .getTagPose(result.targets.get(j).getFiducialId())
                    .get();
          }
          Logger.recordOutput("Vision/" + camera.getName() + "/Target Poses", targetPose3ds);

        } else {
          Logger.recordOutput("Vision/" + camera.getName() + "/Invalid Pose Result", "Stale");
        }
      } catch (NoSuchElementException e) {
        Logger.recordOutput("Vision/" + camera.getName() + "/Invalid Pose Result", "Bad Estimate");
      }
      i++;
    }
    Logger.recordOutput("Vision/Camera Poses", cameraPoses);
    Pose3d[] arr = new Pose3d[cameras.length];
    for (int k = 0; k < cameras.length; k++) {
      arr[k] = getPose3d().transformBy(cameras[k].inputs.constants.robotToCamera());
    }
    Logger.recordOutput("Vision/Camera Poses on Robot", arr);
  }

  /**
   * Generates a set of samples without using the async thread. Makes lots of Objects, so be careful
   * when using it irl!
   */
  private List<Samples> getSyncSamples() {
    return List.of(
        new Samples(
            Logger.getTimestamp() / 1.0e6,
            Map.of(
                new SignalID(SignalType.DRIVE, 0), modules[0].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 0), modules[0].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 1), modules[1].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 1), modules[1].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 2), modules[2].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 2), modules[2].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 3), modules[3].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 3), modules[3].getPosition().angle.getRotations(),
                new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID),
                    gyroInputs.yawPosition.getDegrees())));
  }

  /** Returns the current pose estimator pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  /** Wraps {@link #getPose()} in a Pose3d. */
  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  /** Returns the current odometry rotation as returned by {@link #getPose()}. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void resetPose(Pose2d pose) {
    estimator.resetPose(pose);
    if (simulation.isPresent()) {
      simulation.get().setSimulationWorldPose(pose);
      simulation.get().setRobotSpeeds(new ChassisSpeeds());
    }
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }

  /** Get the drivebase's robot relative chassis speeds */
  @AutoLogOutput(key = "Odometry/Velocity Robot Relative")
  public ChassisSpeeds getVelocityRobotRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /** Get the drivebase's field relative chassis speeds */
  @AutoLogOutput(key = "Odometry/Velocity Field Relative")
  public ChassisSpeeds getVelocityFieldRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation());
    return speeds;
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds robot relative target speeds
   * @param openLoop should the modules use open loop voltage to approximate the setpoint
   * @param moduleForcesX field relative force feedforward to apply to the modules. Must have the
   *     same number of elements as there are modules.
   * @param moduleForcesY field relative force feedforward to apply to the modules. Must have the
   *     same number of elements as there are modules.
   */
  private void drive(
      ChassisSpeeds speeds, boolean openLoop, double[] moduleForcesX, double[] moduleForcesY) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    final SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, constants.getMaxLinearSpeed());

    Logger.recordOutput("Swerve/Target Speeds", speeds);
    Logger.recordOutput("Swerve/Speed Error", speeds.minus(getVelocityRobotRelative()));
    Logger.recordOutput(
        "Swerve/Target Chassis Speeds Field Relative",
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation()));

    // Send setpoints to modules
    final SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[modules.length];
    // Kind of abusing SwerveModuleState here but w/e
    final SwerveModuleState[] forceSetpoints = new SwerveModuleState[modules.length];
    for (int i = 0; i < optimizedSetpointStates.length; i++) {
      if (openLoop) {
        // Use open loop voltage control (teleop)
        // Heuristic to enable/disable FOC
        final boolean focEnable =
            Math.sqrt(
                    Math.pow(this.getVelocityRobotRelative().vxMetersPerSecond, 2)
                        + Math.pow(this.getVelocityRobotRelative().vyMetersPerSecond, 2))
                < constants.getMaxLinearSpeed()
                    * 0.9; // TODO tune the magic number (90% of free speed)
        optimizedSetpointStates[i] =
            modules[i].runVoltageSetpoint(
                new SwerveModuleState(
                    // Convert velocity to voltage with kv
                    setpointStates[i].speedMetersPerSecond * 12.0 / constants.getMaxLinearSpeed(),
                    setpointStates[i].angle),
                focEnable);
        // Convert voltage back to m/s
        optimizedSetpointStates[i].speedMetersPerSecond *= constants.getMaxLinearSpeed() / 12.0;

        // Have to put something here to log properly
        forceSetpoints[i] = new SwerveModuleState();
      } else {
        // Use closed loop current control (automated actions)
        // Calculate robot forces
        var robotRelForceX =
            moduleForcesX[i] * getRotation().unaryMinus().getCos()
                - moduleForcesY[i] * getRotation().unaryMinus().getSin();
        var robotRelForceY =
            moduleForcesX[i] * getRotation().unaryMinus().getSin()
                + moduleForcesY[i] * getRotation().unaryMinus().getCos();
        forceSetpoints[i] =
            new SwerveModuleState(
                Math.hypot(robotRelForceX * 0.5, robotRelForceY * 0.5),
                new Rotation2d(robotRelForceX, robotRelForceY));
        optimizedSetpointStates[i] =
            modules[i].runSetpoint(setpointStates[i], robotRelForceX, robotRelForceY);
      }
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/ForceSetpoints", forceSetpoints);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * This function bypasses the command-based framework because Choreolib handles setting
   * requirements internally. Do NOT use outside of ChoreoLib
   *
   * @return a Consumer that runs the drivebase to follow a SwerveSample with PID feedback, sample
   *     target vel feedforward, and module force feedforward.
   */
  @SuppressWarnings("resource")
  public Consumer<SwerveSample> choreoDriveController() {
    final PIDController xController = new PIDController(5.0, 0.0, 0.0);
    final PIDController yController = new PIDController(5.0, 0.0, 0.0);
    final PIDController thetaController =
        new PIDController(constants.getHeadingVelocityKP(), 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return (sample) -> {
      final var pose = getPose();
      Logger.recordOutput(
          "Choreo/Target Pose",
          new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)));
      Logger.recordOutput(
          "Choreo/Target Speeds Field Relative",
          new ChassisSpeeds(sample.vx, sample.vy, sample.omega));
      var feedback =
          new ChassisSpeeds(
              xController.calculate(pose.getX(), sample.x),
              yController.calculate(pose.getY(), sample.y),
              thetaController.calculate(pose.getRotation().getRadians(), sample.heading));
      var speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              new ChassisSpeeds(sample.vx, sample.vy, sample.omega).plus(feedback), getRotation());
      Logger.recordOutput("Choreo/Feedback + FF Target Speeds Robot Relative", speeds);
      this.drive(
          speeds,
          false,
          useModuleForceFF ? sample.moduleForcesX() : new double[4],
          useModuleForceFF ? sample.moduleForcesY() : new double[4]);
    };
  }

  @SuppressWarnings("resource")
  public Command poseLockDriveCommand(Supplier<Pose2d> targetSupplier) {
    final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    final PIDController thetaController =
        new PIDController(constants.getHeadingVelocityKP(), 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return this.run(
        () -> {
          final var pose = getPose();
          final var target = targetSupplier.get();
          Logger.recordOutput("Swerve/Target Pose", target);
          final var speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      xController.calculate(pose.getX(), target.getX()),
                      yController.calculate(pose.getY(), target.getY()),
                      thetaController.calculate(
                          pose.getRotation().getRadians(), target.getRotation().getRadians())),
                  getRotation());
          Logger.recordOutput("Choreo/Feedback + FF Target Speeds Robot Relative", speeds);
          this.drive(speeds, false, new double[4], new double[4]);
        });
  }

  /**
   * Drive at a robot-relative speed.
   *
   * @param speeds the robot-relative speed reference.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), false, new double[4], new double[4]));
  }

  /**
   * Drive at a robot-relative speed.
   *
   * @param speeds the robot-relative speed reference.
   * @param xForces an array of forces (in Nm) along the field-relative x axis to add as a
   *     feedforward to the modules.
   * @param yForces an array of forces (in Nm) along the field-relative y axis to add as a
   *     feedforward to the modules.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocity(
      Supplier<ChassisSpeeds> speeds, Supplier<double[]> xForces, Supplier<double[]> yForces) {
    return this.run(() -> drive(speeds.get(), false, xForces.get(), yForces.get()));
  }

  /**
   * Drive at a field-relative speed.
   *
   * @param speeds the field-relative speed reference.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.driveVelocity(
        () -> {
          var speed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation());
          return speed;
        });
  }

  /**
   * Drive at a field-relative speed.
   *
   * @param speeds the field-relative speed reference.
   * @param xForces an array of forces (in Nm) along the field-relative x axis to add as a
   *     feedforward to the modules.
   * @param yForces an array of forces (in Nm) along the field-relative y axis to add as a
   *     feedforward to the modules.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocityFieldRelative(
      Supplier<ChassisSpeeds> speeds, Supplier<double[]> xForces, Supplier<double[]> yForces) {
    return this.driveVelocity(
        () -> {
          var speed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation());
          return speed;
        },
        xForces,
        yForces);
  }

  public Command driveTeleop(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          var speed =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          this.drive(speed, true, new double[4], new double[4]);
        });
  }

  public Command driveCharacterization() {
    Timer timer = new Timer();
    return this.run(
            () -> {
              for (Module m : modules) {
                m.setCurrent(timer.get());
                m.setTurnSetpoint(Rotation2d.kZero);
              }
            })
        .beforeStarting(() -> timer.restart());
  }
}
