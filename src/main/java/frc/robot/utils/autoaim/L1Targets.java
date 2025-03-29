package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.List;

public enum L1Targets {
  BLUE_AB(
      new Rectangle2d(
          new Pose2d(3.64, 4.03, Rotation2d.fromDegrees(180)),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  BLUE_CD(
      new Rectangle2d(
          new Pose2d(4.06, 3.31, Rotation2d.fromDegrees(240)),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  BLUE_EF(
      new Rectangle2d(
          new Pose2d(4.89, 3.31, Rotation2d.fromDegrees(300)),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  BLUE_GH(
      new Rectangle2d(
          new Pose2d(5.31, 4.03, Rotation2d.fromDegrees(0)), 0.0, AutoAim.L1_TROUGH_WIDTH_METERS)),
  BLUE_IJ(
      new Rectangle2d(
          new Pose2d(4.89, 4.75, Rotation2d.fromDegrees(60)), 0.0, AutoAim.L1_TROUGH_WIDTH_METERS)),
  BLUE_KL(
      new Rectangle2d(
          new Pose2d(4.06, 4.75, Rotation2d.fromDegrees(120)),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),

  RED_AB(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_AB.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  RED_CD(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_CD.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  RED_EF(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_EF.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  RED_GH(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_GH.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  RED_IJ(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_IJ.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS)),
  RED_KL(
      new Rectangle2d(
          ChoreoAllianceFlipUtil.flip(BLUE_KL.line.getCenter()),
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS));

  public Rectangle2d line;

  private L1Targets(Rectangle2d line) {
    this.line = line;
  }

  private static final List<Rectangle2d> transformedLines =
      Arrays.stream(values())
          .map(
              (L1Targets targets) -> {
                return L1Targets.getRobotTargetLine(targets.line);
              })
          .toList();

  public static Rectangle2d getRobotTargetLine(Rectangle2d original) {
    return original.transformBy(
        new Transform2d(
            Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2,
            0,
            Rotation2d.fromDegrees(180.0)));
  }

  public static Rectangle2d getNearestLine(Pose2d pose) {
    // It feels like there should be a better way to do this
    return new Rectangle2d(
        pose.nearest(transformedLines.stream().map(line -> line.getCenter()).toList()),
        AutoAim.L1_TROUGH_WIDTH_METERS,
        0.0);
  }
}
