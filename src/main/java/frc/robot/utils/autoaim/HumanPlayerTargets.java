package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
