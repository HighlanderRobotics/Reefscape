package frc.robot.utils.gamepiece;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utils.gamepiece.GamePiece.Piece;
import java.io.IOException;
import us.hebi.quickbuf.Descriptors;
import us.hebi.quickbuf.ProtoMessage;
import us.hebi.quickbuf.ProtoSink;
import us.hebi.quickbuf.ProtoSource;
import us.hebi.quickbuf.RepeatedByte;
import edu.wpi.first.math.proto.Geometry3D.ProtobufPose3d;


public final class ProtobufGamePiece extends ProtoMessage<ProtobufGamePiece> implements Cloneable {
  public static final int POSE_FIELD_NUMBER = 1;
  public static final int TYPE_FIELD_NUMBER = 2;

  private static final Descriptors.FileDescriptor FILE_DESCRIPTOR =
      Descriptors.FileDescriptor.internalBuildGeneratedFileFrom(
          "gamepiece.proto", "petro.proto", RepeatedByte.newEmptyInstance());

  private static final Descriptors.Descriptor DESCRIPTOR =
      FILE_DESCRIPTOR.internalContainedType(0, 0, "Game Piece", "ProtobufGamePiece");

//   private Pose3d pose;
  private final ProtobufPose3d poseProto = ProtobufPose3d.newInstance();
  private Pose3d pose = new Pose3d();


  private GamePiece.Piece type;

  private static final ProtobufGamePiece DEFAULT_INSTANCE = new ProtobufGamePiece();

  public static final ProtobufGamePiece getDefaultInstance() {
    return DEFAULT_INSTANCE;
  }

  public static final ProtobufGamePiece newInstance() {
    return new ProtobufGamePiece();
  }

  public static final Descriptors.Descriptor getDescriptor() {
    return DESCRIPTOR;
  }

  public final GamePiece.Piece getType() {
    return type;
  }

  public final ProtobufPose3d getPoseProto() {
    return poseProto;
  }

  public final Pose3d getPose() {
    return pose;
  }

  @Override
  public boolean isInitialized() {
    return true;
  }

  @Override
  public ProtobufGamePiece copyFrom(ProtobufGamePiece other) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'copyFrom'");
  }

  @Override
  public ProtobufGamePiece clear() {
    return ProtobufGamePiece.newInstance();
  }

  @Override
  protected int computeSerializedSize() {
    int size = 0;
    if (pose != null) {
      size += ProtoSink.computeMessageSize(POSE_FIELD_NUMBER, pose.proto.createMessage());
    }
    if (type != null) {
      size += ProtoSink.computeEnumSize(TYPE_FIELD_NUMBER, type.getNumber());
    }
    return size;
  }

  @Override
  public void writeTo(ProtoSink output) throws IOException {
    if (pose != null) {
      output.writeMessage(POSE_FIELD_NUMBER, pose.proto.createMessage());
    }
    if (type != null) {
      output.writeEnum(TYPE_FIELD_NUMBER, type.getNumber());
    }
  }

  @Override
  public ProtobufGamePiece mergeFrom(ProtoSource input) throws IOException {
    while (true) {
      int tag = input.readTag();
      if (tag == 0) {
        break;
      }
      switch (tag) {
        case (POSE_FIELD_NUMBER << 3):
          if (pose == null) {
            pose = new Pose3d();
          }
          input.readMessage(poseProto);
          break;
        case (TYPE_FIELD_NUMBER << 3):
          type = GamePiece.Piece.forNumber(input.readEnum());
          break;
        default:
          if (!input.skipField(tag)) {
            return this;
          }
          break;
      }
    }
    return this;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!(obj instanceof ProtobufGamePiece)) {
      return false;
    }
    ProtobufGamePiece other = (ProtobufGamePiece) obj;
    return (pose == null ? other.pose == null : pose.equals(other.pose))
        && (type == null ? other.type == null : type.equals(other.type));
  }

  @Override
  public ProtobufGamePiece clone() {
    ProtobufGamePiece clone = new ProtobufGamePiece();
    if (pose != null) {
      clone.pose = new Pose3d(pose.getX(), pose.getY(), pose.getZ(), pose.getRotation());
    }
    clone.type = type;
    return clone;
  }

  public void setType(Piece type) {
    this.type = type;
  }
}
