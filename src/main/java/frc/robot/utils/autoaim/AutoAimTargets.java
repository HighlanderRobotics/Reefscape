package frc.robot.utils.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.utils.RobotConstants;

public enum AutoAimTargets {
  // All coordinates are global coordinates from the lower, blue alliance side corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the field, counterclockwise
  BLUE_A(new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(180))),
  BLUE_B(new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(180))),
  BLUE_C(new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(240))),
  BLUE_D(new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(240))),
  BLUE_E(new Pose2d(4.60, 3.50, Rotation2d.fromDegrees(300))),
  BLUE_F(new Pose2d(4.88, 3.66, Rotation2d.fromDegrees(300))),
  BLUE_G(new Pose2d(5.00, 3.90, Rotation2d.fromDegrees(0))),
  BLUE_H(new Pose2d(5.00, 4.20, Rotation2d.fromDegrees(0))),
  BLUE_I(new Pose2d(4.88, 4.41, Rotation2d.fromDegrees(60))),
  BLUE_J(new Pose2d(4.60, 4.57, Rotation2d.fromDegrees(60))),
  BLUE_K(new Pose2d(4.36, 4.57, Rotation2d.fromDegrees(120))),
  BLUE_L(new Pose2d(4.06, 4.41, Rotation2d.fromDegrees(120))),

  RED_A(new Pose2d(13.57, 3.87, Rotation2d.fromDegrees(0))),
  RED_B(new Pose2d(13.57, 4.20, Rotation2d.fromDegrees(0))),
  RED_C(new Pose2d(13.45, 4.40, Rotation2d.fromDegrees(60))),
  RED_D(new Pose2d(13.17, 4.57, Rotation2d.fromDegrees(60))),
  RED_E(new Pose2d(12.92, 4.57, Rotation2d.fromDegrees(120))),
  RED_F(new Pose2d(12.64, 4.41, Rotation2d.fromDegrees(120))),
  RED_G(new Pose2d(12.52, 4.20, Rotation2d.fromDegrees(180))),
  RED_H(new Pose2d(12.52, 3.87, Rotation2d.fromDegrees(180))),
  RED_I(new Pose2d(12.64, 3.66, Rotation2d.fromDegrees(240))),
  RED_J(new Pose2d(12.92, 3.49, Rotation2d.fromDegrees(240))),
  RED_K(new Pose2d(13.17, 3.50, Rotation2d.fromDegrees(300))),
  RED_L(new Pose2d(13.45, 3.66, Rotation2d.fromDegrees(300)));

  public final Pose2d location;

  private AutoAimTargets(edu.wpi.first.math.geometry.Pose2d location) {
    this.location = location;
  }

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    // 0.248 for trough
    return offsetPose(
        original,
        new Transform2d(
            0.248 + ((RobotConstants.LENGTH / 2)) + RobotConstants.BUMPER_SIZE,
            0,
            new Rotation2d(0)));
  }

  public static Pose2d offsetPose(Pose2d original, Transform2d offset) {
    return original.transformBy(offset);
  }
}
