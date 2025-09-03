// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.utils.autoaim.AutoAim;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/** Add your docs here. */
public class FieldUtils {
  public enum AlgaeIntakeTargets {
    // All coordinates are global coordinates from the lower, blue alliance side corner, if the
    // walls
    // were extended beyond the coral station
    // All angles from the center of the coral with 0° across the width of the field,
    // counterclockwise
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
                      Math.abs(
                          pose.getRotation().minus(other.location.getRotation()).getRadians())));
    }
  }

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

  public enum CoralTargets {
    // All coordinates are global coordinates from the lower, blue alliance side corner, if the
    // walls
    // were extended beyond the coral station
    // All angles from the center of the coral with 0° across the width of the field,
    // counterclockwise
    BLUE_A(new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(180)), true),
    BLUE_B(new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(180)), false),
    BLUE_C(new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(240)), true),
    BLUE_D(new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(240)), false),
    BLUE_E(new Pose2d(4.60, 3.50, Rotation2d.fromDegrees(300)), false),
    BLUE_F(new Pose2d(4.88, 3.66, Rotation2d.fromDegrees(300)), true),
    BLUE_G(new Pose2d(5.00, 3.86, Rotation2d.fromDegrees(0)), false),
    BLUE_H(new Pose2d(5.00, 4.18, Rotation2d.fromDegrees(0)), true),
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

    private CoralTargets(Pose2d location, boolean leftHanded) {
      this.location = location;
      this.leftHanded = leftHanded;
    }

    private static final List<Pose2d> transformedPoses =
        Arrays.stream(values())
            .map(
                (CoralTargets targets) -> {
                  return CoralTargets.getRobotTargetLocation(targets.location);
                })
            .toList();

    public static Pose2d getRobotTargetLocation(Pose2d original) {
      // 0.248 for trough
      return original.transformBy(
          new Transform2d(
              0.291 + (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
              0,
              Rotation2d.fromDegrees(180.0)));
    }

    public static Pose2d getBranchLocation(Pose2d transformed) {
      // 0.248 for trough
      return transformed.transformBy(
          new Transform2d(
                  0.291 + (Robot.ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
                  0,
                  Rotation2d.fromDegrees(180.0))
              .inverse());
    }

    /** Gets the closest offset target to the given pose. */
    public static Pose2d getClosestTarget(Pose2d pose) {
      return pose.nearest(transformedPoses);
    }

    /** Gets the closest offset target to the given pose. */
    public static Pose2d getHandedClosestTarget(Pose2d pose, boolean leftHandeed) {
      return pose.nearest(
          Arrays.stream(values())
              .filter((target) -> target.leftHanded == leftHandeed)
              .map(
                  (CoralTargets targets) -> {
                    return CoralTargets.getRobotTargetLocation(targets.location);
                  })
              .toList());
    }
  }

  public enum HumanPlayerTargets {
    BLUE_RIGHT_OUTSIDE(
        new Pose2d(
            1.6333351135253906, 0.6246773600578308, Rotation2d.fromRadians(0.9420001549844138))),
    BLUE_RIGHT_MIDDLE(
        new Pose2d(
            1.1213834285736084, 0.9940196871757507, Rotation2d.fromRadians(0.9940196871757508))),
    BLUE_RIGHT_INSIDE(
        new Pose2d(
            0.5838082432746887, 1.3407007455825806, Rotation2d.fromRadians(0.9420001549844138))),

    BLUE_LEFT_OUTSIDE(
        new Pose2d(
            1.666144609451294, 7.431143760681152, Rotation2d.fromRadians(-0.9350057865774469))),
    BLUE_LEFT_MIDDLE(
        new Pose2d(
            1.179524540901184, 7.083498477935791, Rotation2d.fromRadians(-0.9350057865774469))),
    BLUE_LEFT_INSIDE(
        new Pose2d(
            0.6153400540351868, 6.673182487487793, Rotation2d.fromRadians(-0.9350057865774469))),

    RED_RIGHT_OUTSIDE(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT_OUTSIDE.location)),
    RED_RIGHT_MIDDLE(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT_MIDDLE.location)),
    RED_RIGHT_INSIDE(ChoreoAllianceFlipUtil.flip(BLUE_RIGHT_INSIDE.location)),
    RED_LEFT_OUTSIDE(ChoreoAllianceFlipUtil.flip(BLUE_LEFT_OUTSIDE.location)),
    RED_LEFT_MIDDLE(ChoreoAllianceFlipUtil.flip(BLUE_LEFT_MIDDLE.location)),
    RED_LEFT_INSIDE(ChoreoAllianceFlipUtil.flip(BLUE_LEFT_INSIDE.location));

    public final Pose2d location;

    private HumanPlayerTargets(Pose2d location) {
      this.location = location;
    }
  }

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
            new Pose2d(5.31, 4.03, Rotation2d.fromDegrees(0)),
            0.0,
            AutoAim.L1_TROUGH_WIDTH_METERS)),
    BLUE_IJ(
        new Rectangle2d(
            new Pose2d(4.89, 4.75, Rotation2d.fromDegrees(60)),
            0.0,
            AutoAim.L1_TROUGH_WIDTH_METERS)),
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
          0.0,
          AutoAim.L1_TROUGH_WIDTH_METERS);
    }
  }
}
