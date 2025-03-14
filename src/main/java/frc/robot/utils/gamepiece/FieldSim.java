package frc.robot.utils.gamepiece;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.utils.gamepiece.GamePiece.Piece;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class FieldSim {

  private static ArrayList<GamePiece> gamePieces = new ArrayList<GamePiece>();
  private static int heldPiece = -1;

  // example pieces

  static {
    gamePieces.add(new GamePiece(new Pose3d(3, 2, 0, new Rotation3d()), GamePiece.Piece.ALGAE));

    gamePieces.add(new GamePiece(new Pose3d(4, 2, 0, new Rotation3d()), GamePiece.Piece.ALGAE));

    gamePieces.add(new GamePiece(new Pose3d(5, 2, 0, new Rotation3d()), GamePiece.Piece.ALGAE));

    gamePieces.add(new GamePiece(new Pose3d(6, 2, 0, new Rotation3d()), GamePiece.Piece.ALGAE));

    gamePieces.add(new GamePiece(new Pose3d(7, 3, 0, new Rotation3d()), GamePiece.Piece.ALGAE));
    gamePieces.add(new GamePiece(new Pose3d(4, 4, 3, new Rotation3d()), GamePiece.Piece.CORAL));
    Logger.recordOutput("FieldSim/Held Piece", heldPiece);
  }

  public static void addGamePiece(GamePiece gamePiece) {
    gamePieces.add(gamePiece);
    updateLogging();
  }

  public static void updateLogging() {
    for (int i = 0; i < gamePieces.size(); i++) {
      GamePiece gamePiece = gamePieces.get(i);

      Pose3d pose = gamePiece.getPose();
      System.out.println(pose);
      Logger.recordOutput("FieldSim/GamePiece" + i + "/Pose", pose);
    }
    Logger.recordOutput("FieldSim/Held Piece", heldPiece);
    if (heldPiece != -1) {
      Logger.recordOutput("FieldSim/Held Piece", gamePieces.get(heldPiece).getPose());
      Logger.recordOutput("FieldSim/Type", gamePieces.get(heldPiece).getType());
    }
  }

  public static void updateLogging(Pose3d manipulatorPose) {
    for (int i = 0; i < gamePieces.size(); i++) {
      GamePiece gamePiece = gamePieces.get(i);
      if (gamePiece.isHeld()) {
        gamePiece.setPose(manipulatorPose);
      }
      Pose3d pose = gamePiece.getPose();
      // System.out.println(pose);
      Logger.recordOutput("FieldSim/GamePiece" + i + "/Pose", pose);
    }
    Logger.recordOutput("FieldSim/Held Piece", heldPiece);
    if (heldPiece != -1) {
      Logger.recordOutput("FieldSim/Held Piece", gamePieces.get(heldPiece).getPose());
      Logger.recordOutput("FieldSim/Type", gamePieces.get(heldPiece).getType());
    }
  }

  public static void addGamePiece(Pose3d pose, GamePiece.Piece type) {
    gamePieces.add(new GamePiece(pose, type));
    updateLogging();
  }

  private static int getClosestGamePiece(Pose3d pose) {
    double closestDistance = Double.MAX_VALUE;
    int closestGamePiece = -1;

    for (int i = 0; i < gamePieces.size(); i++) {
      if (gamePieces.get(i).getPose() == null || gamePieces.get(i).getType() == Piece.NONE) {
        //  System.out.println("Game Piece is null");
        continue;
      }

      double distance =
          pose.getTranslation().getDistance(gamePieces.get(i).getPose().getTranslation());
      // System.out.println(distance + " /  " + closestDistance + " / " + i);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestGamePiece = i;
      }
    }

    return closestGamePiece;
  }

  private static boolean inRange(Pose3d pose) {
    int closestGamePiece = getClosestGamePiece(pose);
    if (closestGamePiece != -1) {
      Logger.recordOutput("FieldSim/Manipulator", pose);
      Logger.recordOutput("FieldSim/isNull", false);
      Pose3d closestPose = gamePieces.get(closestGamePiece).getPose();
      Logger.recordOutput("FieldSim/closestGamePiece", closestPose);
      boolean inRange =
          // TODO: change tolerance values from testing
          MathUtil.isNear(closestPose.getX(), pose.getX(), 0.5)
              && MathUtil.isNear(closestPose.getY(), pose.getY(), 0.5)
              && MathUtil.isNear(closestPose.getZ(), pose.getZ(), 0.5);
      Logger.recordOutput("FieldSim/In Range", inRange);
      return inRange;
    } else {
      Logger.recordOutput("FieldSim/isNull", true);
      return false;
    }
  }

  public static int getHeldPiece() {
    return heldPiece;
  }

  public static ArrayList<GamePiece> getGamePieces() {
    return gamePieces;
  }

  public static void pickup(Pose3d pose, ManipulatorSubsystem manipulator) {
    Logger.recordOutput("FieldSim/Pickup", "Pickup");
    int gamePiece = getClosestGamePiece(pose);

    // System.out.println(gamePiece + " game piece");
    if (inRange(pose)) {

      Logger.recordOutput("FieldSim/Type", gamePieces.get(gamePiece).getType());
      updateLogging();
      if (gamePieces.get(gamePiece).getType().equals(GamePiece.Piece.ALGAE)) {
        Logger.recordOutput("FieldSim/Pickup", "Pickup algae");

        manipulator.setHasAlgae(true);

      } else if (gamePieces.get(gamePiece).getType().equals(GamePiece.Piece.CORAL)) {
        Logger.recordOutput("FieldSim/Pickup", "Pickup coral");
        manipulator.setFirstBeambreak(true);
        manipulator.setSecondBeambreak(true);
      } else {
        Logger.recordOutput("FieldSim/Pickup", "Pickup failed");
        return;
      }
      gamePieces.get(gamePiece).pickup();
      heldPiece = gamePiece;

      updateLogging();
    } else {
      Logger.recordOutput("FieldSim/Pickup", "Not in range");
      Logger.recordOutput("FieldSim/range", inRange(pose));
    }
  }

  public static void drop(Pose3d pose, ManipulatorSubsystem manipulator) {
    if (heldPiece == -1) {

      Logger.recordOutput("FieldSim/Drop Status", "Cannot Drop (no held piece)");
      return;
    }

    Logger.recordOutput("FieldSim/Type", gamePieces.get(heldPiece).getType());
    System.out.println("JGEIOHJOJGIOGJEIOGJ");
    GamePiece held = gamePieces.get(heldPiece);

    if (!held.isHeld()) {
      // this should never happen
      Logger.recordOutput("FieldSim/Drop Status", "Cannot Drop (not being held)");
      return;
    }

    if (held.getType().equals(GamePiece.Piece.ALGAE)) {
      manipulator.setHasAlgae(false);

    } else if (held.getType().equals(GamePiece.Piece.CORAL)) {
      manipulator.setFirstBeambreak(false);
      manipulator.setSecondBeambreak(false);
    } else {
      Logger.recordOutput("FieldSim/Drop Status", "Cannot Drop");
      return;
    }

    gamePieces.get(heldPiece).drop(pose);

    heldPiece = -1;
    updateLogging();
    ;
  }
}
