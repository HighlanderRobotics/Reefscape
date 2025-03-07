// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.autoaim.CoralTargets;

public class ExtensionKinematics {
  public static final ExtensionState L2_EXTENSION =
      new ExtensionState(
          ElevatorSubsystem.L2_EXTENSION_METERS,
          ShoulderSubsystem.SHOULDER_SCORE_POS,
          WristSubsystem.WRIST_SCORE_L2_POS);
  public static final ExtensionState L3_EXTENSION =
      new ExtensionState(
          ElevatorSubsystem.L3_EXTENSION_METERS,
          ShoulderSubsystem.SHOULDER_SCORE_POS,
          WristSubsystem.WRIST_SCORE_L3_POS);
  public static final ExtensionState L4_EXTENSION =
      new ExtensionState(
          ElevatorSubsystem.L4_EXTENSION_METERS,
          ShoulderSubsystem.SHOULDER_SCORE_L4_POS,
          WristSubsystem.WRIST_SCORE_L4_POS);

  public record ExtensionState(
      double elevatorHeightMeters, Rotation2d shoulderAngle, Rotation2d wristAngle) {}

  private ExtensionKinematics() {}

  /**
   * @param target pose where +x is robot +x from elevator, +y is robot +z from elevator min, and
   *     rotation is coral angle from horizontal
   */
  public static ExtensionState solveIK(Pose2d target) {
    // Offset wrist pose from target
    final var wristPose = target.transformBy(ManipulatorSubsystem.IK_WRIST_TO_CORAL.inverse());
    // Find shoulder angle from needed horizontal extension
    var shoulderAngle = Math.acos(wristPose.getX() / ShoulderSubsystem.ARM_LENGTH_METERS);
    // Set angle to horizontal if we can't reach
    if (Double.isNaN(shoulderAngle)) shoulderAngle = 0.0;
    // Elevator goes to remaining needed height
    final var elevatorHeight =
        wristPose
            .getTranslation()
            .minus(
                new Translation2d(
                    ShoulderSubsystem.ARM_LENGTH_METERS * Math.cos(shoulderAngle),
                    ShoulderSubsystem.ARM_LENGTH_METERS * Math.sin(shoulderAngle)))
            .getY();
    return new ExtensionState(
        elevatorHeight, Rotation2d.fromRadians(shoulderAngle), wristPose.getRotation());
  }

  public static Pose2d solveFK(ExtensionState state) {
    return new Pose2d(
            state.shoulderAngle().getCos() * ShoulderSubsystem.ARM_LENGTH_METERS,
            state.elevatorHeightMeters()
                + state.shoulderAngle().getSin() * ShoulderSubsystem.ARM_LENGTH_METERS,
            state.wristAngle())
        .transformBy(ManipulatorSubsystem.IK_WRIST_TO_CORAL);
  }

  public static ExtensionState getPoseCompensatedExtension(Pose2d pose, ExtensionState target) {
    final var fk = ExtensionKinematics.solveFK(target);
    final var diff = pose.minus(CoralTargets.getClosestTarget(pose));
    final var adjustedFk = new Pose2d(fk.getX() - diff.getX(), fk.getY(), fk.getRotation());
    return ExtensionKinematics.solveIK(adjustedFk);
  }
}
