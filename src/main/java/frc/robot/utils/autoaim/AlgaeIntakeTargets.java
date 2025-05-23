package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Robot.AlgaeIntakeTarget;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

public enum AlgaeIntakeTargets {
  // All coordinates are global coordinates from the lower, blue alliance side corner, if the walls
  // were extended beyond the coral station
  // All angles from the center of the coral with 0° across the width of the field, counterclockwise
  BLUE_AB(new Pose2d(3.64, 4.03, Rotation2d.fromDegrees(180)), AlgaeIntakeTarget.HIGH),
  BLUE_CD(new Pose2d(4.06, 3.31, Rotation2d.fromDegrees(240)), AlgaeIntakeTarget.LOW),
  BLUE_EF(new Pose2d(4.89, 3.31, Rotation2d.fromDegrees(300)), AlgaeIntakeTarget.HIGH),
  BLUE_GH(new Pose2d(5.31, 4.03, Rotation2d.fromDegrees(0)), AlgaeIntakeTarget.LOW),
  BLUE_IJ(new Pose2d(4.89, 4.75, Rotation2d.fromDegrees(60)), AlgaeIntakeTarget.HIGH),
  BLUE_KL(new Pose2d(4.06, 4.75, Rotation2d.fromDegrees(120)), AlgaeIntakeTarget.LOW),

  RED_AB(ChoreoAllianceFlipUtil.flip(BLUE_AB.location), AlgaeIntakeTarget.HIGH),
  RED_CD(ChoreoAllianceFlipUtil.flip(BLUE_CD.location), AlgaeIntakeTarget.LOW),
  RED_EF(ChoreoAllianceFlipUtil.flip(BLUE_EF.location), AlgaeIntakeTarget.HIGH),
  RED_GH(ChoreoAllianceFlipUtil.flip(BLUE_GH.location), AlgaeIntakeTarget.LOW),
  RED_IJ(ChoreoAllianceFlipUtil.flip(BLUE_IJ.location), AlgaeIntakeTarget.HIGH),
  RED_KL(ChoreoAllianceFlipUtil.flip(BLUE_KL.location), AlgaeIntakeTarget.LOW);

  public final Pose2d location;
  public final AlgaeIntakeTarget height;

  private AlgaeIntakeTargets(Pose2d location, AlgaeIntakeTarget height) {
    this.location = location;
    this.height = height;
  }

  private static final List<Pose2d> transformedPoses =
      Arrays.stream(values())
          .map(
              (AlgaeIntakeTargets targets) -> {
                return AlgaeIntakeTargets.getRobotTargetLocation(targets.location);
              })
          .toList();

  public static Pose2d getRobotTargetLocation(Pose2d original) {
    return original.transformBy(
        new Transform2d(
            (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
            0,
            Rotation2d.fromDegrees(180.0)));
  }

  public static Pose2d getOffsetLocation(Pose2d original) {
    return original.transformBy(
        new Transform2d((-0.3 - Units.inchesToMeters(6)), 0, Rotation2d.kZero));
  }

  /** Gets the closest offset target to the given pose. */
  public static Pose2d getClosestTargetPose(Pose2d pose) {
    return pose.nearest(transformedPoses);
  }

  public static AlgaeIntakeTargets getClosestTarget(Pose2d pose) {
    return Collections.min(
        Stream.of(AlgaeIntakeTargets.values()).toList(),
        Comparator.comparing(
                (AlgaeIntakeTargets other) ->
                    pose.getTranslation().getDistance(other.location.getTranslation()))
            .thenComparing(
                (AlgaeIntakeTargets other) ->
                    Math.abs(pose.getRotation().minus(other.location.getRotation()).getRadians())));
  }
}
