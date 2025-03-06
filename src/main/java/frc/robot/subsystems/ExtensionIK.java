// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;

public class ExtensionIK {
  private ExtensionIK() {}

  public record ExtensionState (double elevatorHeightMeters, Rotation2d shoulderAngle, Rotation2d wristAngle) {}

  /** @param target pose where +x is robot +x from elevator, +y is robot +z from elevator min, and rotation is coral angle from horizontal */
  public static ExtensionState solveIK(Pose2d target) {
    // Offset wrist pose from target
    final var wristPose = target.transformBy(ManipulatorSubsystem.IK_WRIST_TO_CORAL.inverse());
    // Find shoulder angle from needed horizontal extension
    var shoulderAngle = Math.acos(wristPose.getX() / ShoulderSubsystem.ARM_LENGTH_METERS);
    // Set angle to horizontal if we can't reach
    if (shoulderAngle == Double.NaN) shoulderAngle = 0.0;
    // Elevator goes to remaining needed height
    final var elevatorHeight = wristPose.getTranslation().minus(new Translation2d(ShoulderSubsystem.ARM_LENGTH_METERS * Math.cos(shoulderAngle), ShoulderSubsystem.ARM_LENGTH_METERS * Math.sin(shoulderAngle))).getY();
    return new ExtensionState(elevatorHeight, Rotation2d.fromRadians(shoulderAngle), wristPose.getRotation());
  }
}
