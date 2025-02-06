package frc.robot.utils.gamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.ArrayList;
import java.util.Optional;

public class FieldSim implements StructSerializable, ProtobufSerializable {

  private static ArrayList<GamePiece> gamePieces = new ArrayList<GamePiece>();

  // example pieces

  static {
    gamePieces.add(
        new GamePiece(new Pose3d(0, 0, 0, new Rotation3d()), GamePiece.Piece.CORAL));
    gamePieces.add(
        new GamePiece(new Pose3d(1, 1, 1, new Rotation3d()), GamePiece.Piece.ALGAE));
  }

  public static void addGamePiece(GamePiece gamePiece) {
    gamePieces.add(gamePiece);
  }

  public static void addGamePiece(Pose3d pose, GamePiece.Piece type) {
    gamePieces.add(new GamePiece(pose, type));
  }

  public static Optional<GamePiece> getClosestGamePiece(Pose3d pose) {
    double closestDistance = Double.MAX_VALUE;
    GamePiece closestGamePiece = null;

    for (GamePiece gamePiece : gamePieces) {
      double distance = pose.getTranslation().getDistance(gamePiece.getPose().getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestGamePiece = gamePiece;
      }
    }

    return Optional.ofNullable(closestGamePiece);
  }

  public static ArrayList<GamePiece> getGamePieces() {
    return gamePieces;
  }
}
