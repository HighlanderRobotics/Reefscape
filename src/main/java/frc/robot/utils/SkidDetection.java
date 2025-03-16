package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;

public class SkidDetection {

  private SwerveDriveKinematics kinematics;

  public SkidDetection(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
  }

  /**
   * Detects modules that are skidding
   *
   * @param moduleStates [Front left, front right, back left, back right]
   * @param yawVelocityRadPerSec From gyro
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
    // Check that this is the correct calc
    double xTransComponentStdDev = Math.sqrt(Arrays.stream(xTransComponents).map(value -> Math.pow(value - xTransComponentAverage, 2)).sum() / 4);
            
    double yTransComponentAverage = Arrays.stream(yTransComponents).average().getAsDouble();
    double yTransComponentStdDev = Math.sqrt(Arrays.stream(yTransComponents).map(value -> Math.pow(value - yTransComponentAverage, 2)).sum() / 4);

    boolean[] result = new boolean[4];
    for (int i = 0; i <= moduleStates.length; i++) {
      // If the measurement deviates by more than 2 standard deviations assume it's skidding (TODO: TUNE)
      if (Math.abs(xTransComponents[i] - xTransComponentAverage) >= xTransComponentStdDev * 2
          || Math.abs(yTransComponents[i] - yTransComponentAverage) >= yTransComponentStdDev * 2) {
        result[i] = true;
      } else {
        result[i] = false;
      }
    }

    return result;
  }
}
