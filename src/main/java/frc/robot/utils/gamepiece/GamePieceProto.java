package frc.robot.utils.gamepiece;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.protobuf.Protobuf;
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
    return ProtobufGamePiece.newInstance();
  }

  @Override
  public GamePiece unpack(ProtobufGamePiece msg) {
    return new GamePiece(
        Pose3d.proto.unpack(msg.getPoseProto()),
        GamePiece.Piece.valueOf(msg.getType().toString()));
  }

  @Override
  public void pack(ProtobufGamePiece msg, GamePiece value) {
    Pose3d.proto.pack(msg.getPose().proto.createMessage(), value.getPose());
    msg.setType(value.getType());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
