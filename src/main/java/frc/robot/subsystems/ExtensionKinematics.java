// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.autoaim.CoralTargets;
import java.util.function.Supplier;

public class ExtensionKinematics {

  // These need to be here bc their main constants arent loaded when we need the constants in this
  // class
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.5);
  static final Transform2d IK_WRIST_TO_CORAL =
      new Transform2d(
          Units.inchesToMeters(12.0), Units.inchesToMeters(-6.842), Rotation2d.fromDegrees(0.0));
  private static final double MAX_EXTENSION_METERS = Units.inchesToMeters(63.50);

  // Not super accurate bc of whack
  public static final Pose2d L1_POSE =
      new Pose2d(0.26, 0.4, Rotation2d.fromDegrees(15.0)); // solveFK(L1_EXTENSION);
  public static final ExtensionState L1_EXTENSION = solveIK(L1_POSE);
  public static final ExtensionState L2_EXTENSION =
      new ExtensionState(
          0.23 + Units.inchesToMeters(1.5),
          Rotation2d.fromRadians(0.569).plus(Rotation2d.fromDegrees(20)),
          Rotation2d.fromRadians(2.447));
  public static final Pose2d L2_POSE = solveFK(L2_EXTENSION);
  public static final ExtensionState L3_EXTENSION =
      new ExtensionState(
          0.60 + Units.inchesToMeters(2.0),
          Rotation2d.fromRadians(1.022).minus(Rotation2d.fromDegrees(3)),
          Rotation2d.fromRadians(2.427));
  public static final Pose2d L3_POSE = solveFK(L3_EXTENSION);
  public static final Pose2d L4_POSE =
      new Pose2d(new Translation2d(0.20, 2.03), Rotation2d.fromDegrees(110.0));

  public static final ExtensionState L4_EXTENSION = solveIK(L4_POSE);

  public static final ExtensionState LOW_ALGAE_EXTENSION =
      new ExtensionState(
          ElevatorSubsystem.INTAKE_ALGAE_LOW_EXTENSION,
          ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_POS,
          WristSubsystem.WRIST_INTAKE_ALGAE_REEF_POS);
  public static final Pose2d LOW_ALGAE_POSE = solveFK(LOW_ALGAE_EXTENSION);

  public static final ExtensionState HIGH_ALGAE_EXTENSION =
      new ExtensionState(
          ElevatorSubsystem.INTAKE_ALGAE_HIGH_EXTENSION,
          ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_POS,
          WristSubsystem.WRIST_INTAKE_ALGAE_REEF_POS);
  public static final Pose2d HIGH_ALGAE_POSE = solveFK(HIGH_ALGAE_EXTENSION);

  public record ExtensionState(
      double elevatorHeightMeters, Rotation2d shoulderAngle, Rotation2d wristAngle) {}

  private ExtensionKinematics() {}

  /**
   * @param target pose where +x is robot +x from elevator, +y is robot +z from elevator min, and
   *     rotation is coral angle from horizontal
   */
  public static ExtensionState solveIK(Pose2d target) {
    // Offset wrist pose from target
    final var wristPose = target.transformBy(IK_WRIST_TO_CORAL.inverse());
    // Find shoulder angle from needed horizontal extension
    var shoulderAngle = Math.acos(wristPose.getX() / ARM_LENGTH_METERS);
    // Set angle to horizontal if we can't reach
    if (Double.isNaN(shoulderAngle)) shoulderAngle = 0.0;
    // Elevator goes to remaining needed height
    var elevatorHeight =
        wristPose
            .getTranslation()
            .minus(
                new Translation2d(
                    ARM_LENGTH_METERS * Math.cos(shoulderAngle),
                    ARM_LENGTH_METERS * Math.sin(shoulderAngle)))
            .getY();
    // If we're extending higher than we can reach, prioritize matching Z instead of X
    if (elevatorHeight > MAX_EXTENSION_METERS) {
      elevatorHeight = MAX_EXTENSION_METERS - Units.inchesToMeters(1.0);
      shoulderAngle = Math.asin((wristPose.getY() - MAX_EXTENSION_METERS) / ARM_LENGTH_METERS);
      // Limit shoulder angle
      if (Double.isNaN(shoulderAngle) || shoulderAngle > Units.degreesToRadians(85.0)) {
        shoulderAngle = Units.degreesToRadians(85.0);
      }
    }

    return new ExtensionState(
        MathUtil.clamp(elevatorHeight, 0.0, ElevatorSubsystem.MAX_EXTENSION_METERS),
        Rotation2d.fromRadians(shoulderAngle),
        Rotation2d.fromDegrees(MathUtil.clamp(wristPose.getRotation().getDegrees(), -45.0, 120.0)));
  }

  public static Pose2d solveFK(ExtensionState state) {
    return new Pose2d(
            state.shoulderAngle().getCos() * ARM_LENGTH_METERS,
            state.elevatorHeightMeters() + state.shoulderAngle().getSin() * ARM_LENGTH_METERS,
            state.wristAngle())
        .transformBy(IK_WRIST_TO_CORAL);
  }

  public static ExtensionState getPoseCompensatedExtension(Pose2d pose, ExtensionState target) {
    final var fk = ExtensionKinematics.solveFK(target);
    final var diff = pose.minus(CoralTargets.getClosestTarget(pose));
    final var adjustedFk = new Pose2d(fk.getX() - diff.getX(), fk.getY(), fk.getRotation());
    return ExtensionKinematics.solveIK(adjustedFk);
  }

  public static Pose3d getManipulatorPose(Pose2d robotPose, ExtensionState state) {
    final var fk = solveFK(state);
    return new Pose3d(
            robotPose.transformBy(
                new Transform2d(
                    fk.getX() + ElevatorSubsystem.X_OFFSET_METERS, 0.0, Rotation2d.kZero)))
        .transformBy(
            new Transform3d(
                0,
                0,
                fk.getY() + ElevatorSubsystem.Z_OFFSET_METERS,
                new Rotation3d(0, -state.wristAngle().getRadians(), 0)));
  }

  public static Pose3d getBranchPose(Pose2d pose, ExtensionState state, ReefTarget level) {
    return new Pose3d(CoralTargets.getClosestTarget(pose))
        .transformBy(
            new Transform3d(
                ElevatorSubsystem.X_OFFSET_METERS
                    + switch (level) {
                      case L1 -> L1_POSE.getX();
                      case L2 -> L2_POSE.getX();
                      case L3 -> L3_POSE.getX();
                      case L4 -> L4_POSE.getX();
                    },
                0,
                ElevatorSubsystem.Z_OFFSET_METERS
                    + switch (level) {
                      case L1 -> L1_POSE.getY();
                      case L2 -> L2_POSE.getY();
                      case L3 -> L3_POSE.getY();
                      case L4 -> L4_POSE.getY();
                    },
                new Rotation3d()));
  }

  public static Command holdStateCommand(
      ElevatorSubsystem elevator,
      ShoulderSubsystem shoulder,
      WristSubsystem wrist,
      Supplier<ExtensionState> target) {
    final LinearFilter elevatorFilter = LinearFilter.movingAverage(8);
    final LinearFilter shoulderFilter = LinearFilter.movingAverage(8);
    final LinearFilter wristFilter = LinearFilter.movingAverage(8);
    return Commands.runOnce(
            () -> {
              elevatorFilter.reset(
                  new double[] {
                    // i hate java surely theres a better way to do this
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters(),
                    elevator.getExtensionMeters()
                  },
                  new double[0]);
              shoulderFilter.reset(
                  new double[] {
                    // i hate java surely theres a better way to do this
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations(),
                    shoulder.getAngle().getRotations()
                  },
                  new double[0]);
              wristFilter.reset(
                  new double[] {
                    // i hate java surely theres a better way to do this
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations(),
                    wrist.getAngle().getRotations()
                  },
                  new double[0]);
            })
        .andThen(
            Commands.parallel(
                elevator.setExtension(
                    () -> elevatorFilter.calculate(target.get().elevatorHeightMeters())),
                shoulder.setTargetAngle(
                    () ->
                        Rotation2d.fromRotations(
                            shoulderFilter.calculate(target.get().shoulderAngle().getRotations()))),
                wrist.setTargetAngle(
                    () ->
                        Rotation2d.fromRotations(
                            wristFilter.calculate(target.get().wristAngle().getRotations())))));
  }

  public static ExtensionState getExtensionForLevel(ReefTarget target) {
    return switch (target) {
      case L2 -> ExtensionKinematics.L2_EXTENSION;
      case L3 -> ExtensionKinematics.L3_EXTENSION;
      case L4 -> ExtensionKinematics.L4_EXTENSION;
      default -> // shouldnt be reachable
      new ExtensionState(
          ElevatorSubsystem.L1_EXTENSION_METERS,
          ShoulderSubsystem.SHOULDER_SCORE_L1_POS,
          WristSubsystem.WRIST_SCORE_L1_POS);
    };
  }
}
