package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.List;

public enum CageTargets {
  RED_OUTSIDE(new Pose2d(8.760, 0.799, Rotation2d.fromDegrees(0)), Alliance.Red),
  RED_MIDDLE(new Pose2d(8.760, 1.889 + 0.15, Rotation2d.fromDegrees(0)), Alliance.Red),
  RED_INSIDE(new Pose2d(8.760, 2.980, Rotation2d.fromDegrees(0)), Alliance.Red),

  BLUE_OUTSIDE(ChoreoAllianceFlipUtil.flip(RED_OUTSIDE.getLocation()), Alliance.Blue),
  BLUE_MIDDLE(ChoreoAllianceFlipUtil.flip(RED_MIDDLE.getLocation()), Alliance.Blue),
  BLUE_INSIDE(ChoreoAllianceFlipUtil.flip(RED_INSIDE.getLocation()), Alliance.Blue);

  private static final List<Pose2d> poses =
      Arrays.stream(values()).map((CageTargets target) -> target.getLocation()).toList();

  private final Pose2d location;
  private final Alliance alliance;

  private CageTargets(Pose2d location, Alliance alliance) {
    this.location = location;
    this.alliance = alliance;
  }

  public static Pose2d getOffsetClosestTarget(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent()) {
      // If it's across the field, x > 8.76 on blue and x < 8.76 on red
      return getOffsetClosestTarget(
          robotPose,
          (DriverStation.getAlliance().get() == Alliance.Blue && robotPose.getX() > 8.76)
              || (DriverStation.getAlliance().get() == Alliance.Red && robotPose.getX() < 8.76));
    }
    return getOffsetClosestTarget(robotPose, false);
  }

  public static Pose2d getOffsetClosestTarget(Pose2d robotPose, boolean far) {
    Pose2d nearestPose;
    if (DriverStation.getAlliance().isPresent()) {
      nearestPose =
          robotPose.nearest(
              Arrays.stream(values())
                  .filter(target -> target.getAlliance() == DriverStation.getAlliance().get())
                  .map(target -> target.getLocation())
                  .toList());
    } else {
      nearestPose = robotPose.nearest(poses);
    }
    if (far) {
      return getFarRobotTargetLocation(nearestPose);
    } else {
      return getCloseRobotTargetLocation(nearestPose);
    }
  }

  public static Pose2d getCloseRobotTargetLocation(Pose2d pose) {
    return pose.transformBy(
        new Transform2d(
            (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2) + 0.087,
            0,
            Rotation2d.kZero));
  }

  public static Pose2d getFarRobotTargetLocation(Pose2d pose) {
    return pose.transformBy(
        new Transform2d(
            -(Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2) - 0.087,
            0,
            Rotation2d.k180deg));
  }

  public Pose2d getLocation() {
    return this.location;
  }

  public Alliance getAlliance() {
    return this.alliance;
  }
}
