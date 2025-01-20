package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import java.util.Arrays;

public enum AutoAimTargets {
  // All coordinates are global coordinates from the lower, blue alliance side corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the field, counterclockwise
  BLUE_A(new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(180)), true),
  BLUE_B(new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(180)), false),
  BLUE_C(new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(240)), true),
  BLUE_D(new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(240)), false),
  BLUE_E(new Pose2d(4.60, 3.50, Rotation2d.fromDegrees(300)), false),
  BLUE_F(new Pose2d(4.88, 3.66, Rotation2d.fromDegrees(300)), true),
  BLUE_G(new Pose2d(5.00, 3.90, Rotation2d.fromDegrees(0)), false),
  BLUE_H(new Pose2d(5.00, 4.20, Rotation2d.fromDegrees(0)), true),
  BLUE_I(new Pose2d(4.88, 4.41, Rotation2d.fromDegrees(60)), false),
  BLUE_J(new Pose2d(4.60, 4.57, Rotation2d.fromDegrees(60)), true),
  BLUE_K(new Pose2d(4.36, 4.57, Rotation2d.fromDegrees(120)), true),
  BLUE_L(new Pose2d(4.06, 4.41, Rotation2d.fromDegrees(120)), false),

  RED_A(ChoreoAllianceFlipUtil.flip(BLUE_A.location), true),
  RED_B(ChoreoAllianceFlipUtil.flip(BLUE_B.location), false),
  RED_C(ChoreoAllianceFlipUtil.flip(BLUE_C.location), true),
  RED_D(ChoreoAllianceFlipUtil.flip(BLUE_D.location), false),
  RED_E(ChoreoAllianceFlipUtil.flip(BLUE_E.location), false),
  RED_F(ChoreoAllianceFlipUtil.flip(BLUE_F.location), true),
  RED_G(ChoreoAllianceFlipUtil.flip(BLUE_G.location), false),
  RED_H(ChoreoAllianceFlipUtil.flip(BLUE_H.location), true),
  RED_I(ChoreoAllianceFlipUtil.flip(BLUE_I.location), false),
  RED_J(ChoreoAllianceFlipUtil.flip(BLUE_J.location), true),
  RED_K(ChoreoAllianceFlipUtil.flip(BLUE_K.location), true),
  RED_L(ChoreoAllianceFlipUtil.flip(BLUE_L.location), false);

  public final Pose2d location;
  public final boolean leftHanded;

  private AutoAimTargets(Pose2d location, boolean leftHanded) {
    this.location = location;
    this.leftHanded = leftHanded;
  }

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    // 0.248 for trough
    return original.transformBy(
        new Transform2d(
            0.291 + (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
            0,
            Rotation2d.fromDegrees(180.0)));
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getClosestTarget(Pose2d pose) {
    return pose.nearest(
        Arrays.stream(values())
            .map(
                (AutoAimTargets targets) -> {
                  return AutoAimTargets.getRobotTargetLocation(targets.location);
                })
            .toList());
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getHandedClosestTarget(Pose2d pose, boolean leftHandeed) {
    return pose.nearest(
        Arrays.stream(values())
            .filter((target) -> target.leftHanded == leftHandeed)
            .map(
                (AutoAimTargets targets) -> {
                  return AutoAimTargets.getRobotTargetLocation(targets.location);
                })
            .toList());
  }
}
