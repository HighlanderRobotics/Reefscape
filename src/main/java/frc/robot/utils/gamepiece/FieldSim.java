package frc.robot.utils.gamepiece;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.utils.gamepiece.GamePiece.Piece;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class FieldSim implements StructSerializable, ProtobufSerializable {

  private static ArrayList<GamePiece> gamePieces = new ArrayList<GamePiece>();
  private static GamePiece heldGamePiece = null;

  // example pieces

  static {
    gamePieces.add(new GamePiece(new Pose3d(3, 3, 0, new Rotation3d()), GamePiece.Piece.CORAL));
    gamePieces.add(new GamePiece(new Pose3d(4, 4, 0, new Rotation3d()), GamePiece.Piece.ALGAE));
    Logger.recordOutput("FieldSim/Held Piece", heldGamePiece);
  }

  public static void addGamePiece(GamePiece gamePiece) {
    gamePieces.add(gamePiece);
    updateLogging();
  }

  public static void updateLogging() {
    for (int i = 0; i < gamePieces.size(); i++) {
      GamePiece gamePiece = gamePieces.get(i);
      Pose3d pose = gamePiece.getPose();
      Logger.recordOutput("FieldSim/GamePiece" + i + "/Pose", pose);
    }
  }

  public static void addGamePiece(Pose3d pose, GamePiece.Piece type) {
    gamePieces.add(new GamePiece(pose, type));
    updateLogging();
  }

  private static Optional<GamePiece> getClosestGamePiece(Pose3d pose) {
    double closestDistance = Double.MAX_VALUE;
    GamePiece closestGamePiece = null;

    for (GamePiece gamePiece : gamePieces) {
      if (gamePiece.getPose() == null) {
        continue;
      }
      double distance = pose.getTranslation().getDistance(gamePiece.getPose().getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestGamePiece = gamePiece;
      }
    }

    return Optional.ofNullable(closestGamePiece);
  }

  private static boolean inRange(Pose3d pose) {
    GamePiece closestGamePiece = getClosestGamePiece(pose).orElse(null);
    if (closestGamePiece != null) {
      Logger.recordOutput("FieldSim/Manipulator", pose);
      Logger.recordOutput("FieldSim/isNull", false);
      Logger.recordOutput("FieldSim/closestGamePiece", closestGamePiece.getPose());
      return MathUtil.isNear(closestGamePiece.getPose().getX(), pose.getX(), 0.5)
          && MathUtil.isNear(closestGamePiece.getPose().getY(), pose.getY(), 0.5)
          && MathUtil.isNear(closestGamePiece.getPose().getZ(), pose.getZ(), 0.5);
    } else {
      Logger.recordOutput("FieldSim/isNull", true);
      return false;
    }
  }

  public static GamePiece getHeldGamePiece() {
    return heldGamePiece;
  }

  public static ArrayList<GamePiece> getGamePieces() {
    return gamePieces;
  }

  public static void pickup(Pose3d pose, ManipulatorSubsystem manipulator) {
    Logger.recordOutput("FieldSim/Pickup", "Pickup");
    getClosestGamePiece(pose)
        .ifPresent(
            gamePiece -> {
              if (inRange(pose)) {
                heldGamePiece = gamePiece;
                gamePiece.pickup();

                if (gamePiece.getType().equals(GamePiece.Piece.ALGAE)) {
                  manipulator.setHasAlgae(true);

                } else {
                  manipulator.setFirstBeambreak(true);
                  manipulator.setSecondBeambreak(true);
                }
                updateLogging();
              } else {
                Logger.recordOutput("FieldSim/Pickup", "Not in range");
                Logger.recordOutput("FieldSim/range", inRange(pose));
              }
            });
  }

  public static void drop(Pose3d pose, ManipulatorSubsystem manipulator) {

    if (heldGamePiece == null) {
      return;
    }
    heldGamePiece.drop(pose);
    if (heldGamePiece.getType().equals(GamePiece.Piece.ALGAE)) {
      manipulator.setHasAlgae(false);

    } else {
      manipulator.setFirstBeambreak(false);
      manipulator.setSecondBeambreak(false);
    }
    heldGamePiece = new GamePiece(pose, Piece.NONE);
    updateLogging();
    ;
  }
}
