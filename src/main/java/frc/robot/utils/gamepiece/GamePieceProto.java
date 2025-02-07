package frc.robot.utils.gamepiece;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.Protobuf;
import org.littletonrobotics.junction.Logger;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class GamePieceProto implements Protobuf<GamePiece, ProtobufGamePiece> {

  @Override
  public Class<GamePiece> getTypeClass() {
    return GamePiece.class;
  }

  @Override
  public Descriptor getDescriptor() {

    return ProtobufGamePiece.getDescriptor();
  }

  @Override
  public ProtobufGamePiece createMessage() {
    Logger.recordOutput("FieldSim/ProtobufGamePiece", new Pose3d());
    return ProtobufGamePiece.newInstance();
  }

  @Override
  public GamePiece unpack(ProtobufGamePiece msg) {
    // Logger.recordOutput("FieldSim/ProtobufGamePiece", msg.getPose());
    Logger.recordOutput("FieldSim/unpack", true);
    GamePiece piece = new GamePiece(Pose3d.proto.unpack(msg.getPoseProto()), msg.getType());
    Logger.recordOutput("FieldSim/GamePiece", piece.getPose());
    return piece;
  }

  @Override
  public void pack(ProtobufGamePiece msg, GamePiece value) {
    Pose3d.proto.pack(msg.getPoseProto(), value.getPose());
    msg.setType(value.getType());

    Logger.recordOutput("FieldSim/pack", msg.toString());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
