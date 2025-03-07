package frc.robot.utils.gamepiece;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class GamePiece implements StructSerializable, ProtobufSerializable {

  // TODO: make serizliable (not difficult)
  public enum Piece {
    NONE,
    CORAL,
    ALGAE;

    public int getNumber() {
      switch (this) {
        case CORAL:
          return 1;
        case ALGAE:
          return 2;
        default:
          return 0;
      }
    }

    public static Piece forNumber(int value) {
      switch (value) {
        case 1:
          return CORAL;
        case 2:
          return ALGAE;
        default:
          return NONE;
      }
    }
  }

  private Pose3d pose;
  private Piece type;
  private boolean isHeld = false;

  public GamePiece(Pose3d pose, Piece type) {
    this.pose = pose;
    this.type = type;
  }

  public boolean isHeld() {
    return isHeld;
  }

  public Pose3d getPose() {
    return pose;
  }

  public Piece getType() {
    return type;
  }

  public void pickup() {
    pose = null;
    isHeld = true;
  }

  public void drop(Pose3d pose) {
    this.pose = pose;
    isHeld = false;
  }

  public String toString() {
    return "GamePiece {" + "pose =" + pose + ", typ e=" + type + '}';
  }
}
