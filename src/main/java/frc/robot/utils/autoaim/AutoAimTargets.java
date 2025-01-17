package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.function.Supplier;

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

  RED_A(ChoreoAllianceFlipUtil.flip(BLUE_A.location)),
  RED_B(ChoreoAllianceFlipUtil.flip(BLUE_B.location)),
  RED_C(ChoreoAllianceFlipUtil.flip(BLUE_C.location)),
  RED_D(ChoreoAllianceFlipUtil.flip(BLUE_D.location)),
  RED_E(ChoreoAllianceFlipUtil.flip(BLUE_E.location)),
  RED_F(ChoreoAllianceFlipUtil.flip(BLUE_F.location)),
  RED_G(ChoreoAllianceFlipUtil.flip(BLUE_G.location)),
  RED_H(ChoreoAllianceFlipUtil.flip(BLUE_H.location)),
  RED_I(ChoreoAllianceFlipUtil.flip(BLUE_I.location)),
  RED_J(ChoreoAllianceFlipUtil.flip(BLUE_J.location)),
  RED_K(ChoreoAllianceFlipUtil.flip(BLUE_K.location)),
  RED_L(ChoreoAllianceFlipUtil.flip(BLUE_L.location));

  public final Pose2d location;

  private AutoAimTargets(edu.wpi.first.math.geometry.Pose2d location) {
    this.location = location;
  }

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    // 0.248 for trough
    return offsetPose(
        original,
        new Transform2d(
            0.248 + (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
            0,
            new Rotation2d(0)));
  }

  public static Pose2d offsetPose(Pose2d original, Transform2d offset) {
    return original.transformBy(offset);
  }

  public static Pose2d getClosestTarget(Supplier<Pose2d> pose) {
    return pose.get()
        .nearest(
            Arrays.stream(values())
                .map(
                    (AutoAimTargets targets) -> {
                      return AutoAimTargets.getRobotTargetLocation(targets.location);
                    })
                .toList());
  }
}
