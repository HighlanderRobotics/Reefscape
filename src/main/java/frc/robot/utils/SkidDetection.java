package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;

public class SkidDetection {

  private SwerveDriveKinematics kinematics;

  // TODO: TUNE
  private static final double MAX_DEVIATION = 0.0;

  public SkidDetection(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
  }

  /**
   * Detects modules that are skidding
   *
   * @param moduleStates [Front left, front right, back left, back right]
   * @param heading
   * @return the skidding modules [Front left, front right, back left, back right] true if skidding
   */
  public boolean[] detectSkiddingModules(SwerveModuleState[] moduleStates, double yawVelocityRadPerSec) {

    // Get the pure rotational component based on the gyro yaw velocity
    SwerveModuleState[] rotationalComponents =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, yawVelocityRadPerSec));
    double[] xTransComponents = new double[4];
    double[] yTransComponents = new double[4];
    for (int i = 0; i <= moduleStates.length; i++) {
      SwerveModuleState moduleState = moduleStates[i];
      SwerveModuleState rotationalComponentState = rotationalComponents[i];
      // Subtract rotational component vectors from the total measured vectors
      xTransComponents[i] =
          (moduleState.speedMetersPerSecond * moduleState.angle.getCos())
              - (rotationalComponentState.speedMetersPerSecond
                  * rotationalComponentState.angle.getCos());
      yTransComponents[i] =
          (moduleState.speedMetersPerSecond * moduleState.angle.getSin())
              - (rotationalComponentState.speedMetersPerSecond
                  * rotationalComponentState.angle.getSin());
    }
    double xTransComponentAverage = Arrays.stream(xTransComponents).average().getAsDouble();
            
    double yTransComponentAverage = Arrays.stream(yTransComponents).average().getAsDouble();

    boolean[] result = new boolean[4];
    for (int i = 0; i <= moduleStates.length; i++) {
      // If the measurement deviates too much from the mean, assume it's skidding
      if (Math.abs(xTransComponents[i] - xTransComponentAverage) > MAX_DEVIATION
          || Math.abs(yTransComponents[i] - yTransComponentAverage) > MAX_DEVIATION) {
        result[i] = true;
      } else {
        result[i] = false;
      }
    }

    return result;
  }
}
