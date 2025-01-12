package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AutoAimTargets {
  // All coordinates are global coordinates from the lower, blue alliance side corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the field, counterclockwise
  BLUE_A(new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(90))),
  BLUE_B(new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(90))),
  BLUE_C(new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(150))),
  BLUE_D(new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(150))),
  BLUE_E(new Pose2d(4.60, 3.50, Rotation2d.fromDegrees(210))),
  BLUE_F(new Pose2d(4.88, 3.66, Rotation2d.fromDegrees(210))),
  BLUE_G(new Pose2d(5.00, 3.90, Rotation2d.fromDegrees(270))),
  BLUE_H(new Pose2d(5.00, 4.20, Rotation2d.fromDegrees(270))),
  BLUE_I(new Pose2d(4.88, 4.41, Rotation2d.fromDegrees(330))),
  BLUE_J(new Pose2d(4.60, 4.57, Rotation2d.fromDegrees(330))),
  BLUE_K(new Pose2d(4.36, 4.57, Rotation2d.fromDegrees(30))),
  BLUE_L(new Pose2d(4.06, 4.41, Rotation2d.fromDegrees(30))),

  RED_A(new Pose2d(13.57, 3.87, Rotation2d.fromDegrees(270))),
  RED_B(new Pose2d(13.57, 4.20, Rotation2d.fromDegrees(270))),
  RED_C(new Pose2d(13.45, 4.40, Rotation2d.fromDegrees(330))),
  RED_D(new Pose2d(13.17, 4.57, Rotation2d.fromDegrees(330))),
  RED_E(new Pose2d(12.92, 4.57, Rotation2d.fromDegrees(30))),
  RED_F(new Pose2d(12.64, 4.41, Rotation2d.fromDegrees(30))),
  RED_G(new Pose2d(12.52, 4.20, Rotation2d.fromDegrees(90))),
  RED_H(new Pose2d(12.52, 3.87, Rotation2d.fromDegrees(90))),
  RED_I(new Pose2d(12.64, 3.66, Rotation2d.fromDegrees(150))),
  RED_J(new Pose2d(12.92, 3.49, Rotation2d.fromDegrees(150))),
  RED_K(new Pose2d(13.17, 3.50, Rotation2d.fromDegrees(210))),
  RED_L(new Pose2d(13.45, 3.66, Rotation2d.fromDegrees(210)));

  public final Pose2d location;

  private AutoAimTargets(edu.wpi.first.math.geometry.Pose2d location) {
    this.location = location;
  }

  public Pose2d getRobotTargetLocation(Pose2d original) {
    // 0.305 for trough
    double requiredTranslation = 0.305 + (RobotConstants.BUMPER_SIZE / 2);
    double rotationRads = original.getRotation().getRadians();
    double xTranslation = requiredTranslation * Math.cos(rotationRads);
    double yTranslation = requiredTranslation * Math.sin(rotationRads);
    return new Pose2d(original.getX() + xTranslation, original.getY() + yTranslation, original.getRotation());
  }
}
