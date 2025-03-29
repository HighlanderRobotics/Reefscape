package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum L1Targets {

    BLUE_AB(new Rectangle2d(new Pose2d(3.64, 4.03, Rotation2d.fromDegrees(180)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    BLUE_CD(new Rectangle2d(new Pose2d(4.06, 3.31, Rotation2d.fromDegrees(240)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    BLUE_EF(new Rectangle2d(new Pose2d(4.89, 3.31, Rotation2d.fromDegrees(300)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    BLUE_GH(new Rectangle2d(new Pose2d(5.31, 4.03, Rotation2d.fromDegrees(0)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    BLUE_IJ(new Rectangle2d(new Pose2d(4.89, 4.75, Rotation2d.fromDegrees(60)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    BLUE_KL(new Rectangle2d(new Pose2d(4.06, 4.75, Rotation2d.fromDegrees(120)), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),

    RED_AB(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_AB.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    RED_CD(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_CD.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    RED_EF(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_EF.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    RED_GH(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_GH.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    RED_IJ(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_IJ.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0)),
    RED_KL(new Rectangle2d(ChoreoAllianceFlipUtil.flip(BLUE_KL.line.getCenter()), AutoAim.L1_TROUGH_WIDTH_METERS, 0.0));

    private Rectangle2d line;

    private L1Targets(Rectangle2d line) {
        this.line = line;
    }
    
}
