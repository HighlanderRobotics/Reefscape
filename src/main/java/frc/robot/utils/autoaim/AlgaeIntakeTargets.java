package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.List;

public enum AlgaeIntakeTargets {
  // All coordinates are global coordinates from the lower, blue alliance side corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the field, counterclockwise
  BLUE_AB(new Pose2d(3.64, 4.03, Rotation2d.fromDegrees(180))),
  BLUE_CD(new Pose2d(4.06, 3.31, Rotation2d.fromDegrees(240))),
  BLUE_EF(new Pose2d(4.89, 3.31, Rotation2d.fromDegrees(300))),
  BLUE_GH(new Pose2d(5.31, 4.03, Rotation2d.fromDegrees(0))),
  BLUE_IJ(new Pose2d(4.89, 4.75, Rotation2d.fromDegrees(60))),
  BLUE_KL(new Pose2d(4.06, 4.75, Rotation2d.fromDegrees(120))),

  RED_AB(ChoreoAllianceFlipUtil.flip(BLUE_AB.location)),
  RED_CD(ChoreoAllianceFlipUtil.flip(BLUE_CD.location)),
  RED_EF(ChoreoAllianceFlipUtil.flip(BLUE_EF.location)),
  RED_GH(ChoreoAllianceFlipUtil.flip(BLUE_GH.location)),
  RED_IJ(ChoreoAllianceFlipUtil.flip(BLUE_IJ.location)),
  RED_KL(ChoreoAllianceFlipUtil.flip(BLUE_KL.location));

  public final Pose2d location;

  private AlgaeIntakeTargets(Pose2d location) {
    this.location = location;
  }

  private static final List<Pose2d> transformedPoses =
      Arrays.stream(values())
          .map(
              (AlgaeIntakeTargets targets) -> {
                return CoralTargets.getRobotTargetLocation(targets.location);
              })
          .toList();

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    return original.transformBy(
        new Transform2d(
            (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
            0,
            Rotation2d.fromDegrees(180.0)));
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getClosestTarget(Pose2d pose) {
    return pose.nearest(transformedPoses);
  }
}
