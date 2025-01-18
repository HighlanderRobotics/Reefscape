package frc.robot.utils.autoaim;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAim {
  static final double MAX_ANGULAR_SPEED = 10.0;
  static final double MAX_ANGULAR_ACCELERATION = 5.0;
  static final double MAX_AUTOAIM_SPEED = 3.0;
  static final double MAX_AUTOAIM_ACCELERATION = 5.0;

  public static Command translateToPose(SwerveSubsystem swerve, Supplier<Pose2d> target) {
    // This feels like a horrible way of getting around lambda final requirements
    // Is there a cleaner way of doing this?
    final Pose2d cachedTarget[] = {new Pose2d()};
    final ProfiledPIDController headingController =
        // assume we can accelerate to max in 2/3 of a second
        new ProfiledPIDController(
            Robot.ROBOT_HARDWARE.swerveConstants.getHeadingVelocityKP(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    final ProfiledPIDController vxController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    final ProfiledPIDController vyController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    return Commands.runOnce(
            () -> {
              cachedTarget[0] = target.get();
              Logger.recordOutput("AutoAim/Cached Target", cachedTarget[0]);
              headingController.reset(
                  swerve.getPose().getRotation().getRadians(),
                  swerve.getVelocityFieldRelative().omegaRadiansPerSecond);
              vxController.reset(
                  swerve.getPose().getX(), swerve.getVelocityFieldRelative().vxMetersPerSecond);
              vyController.reset(
                  swerve.getPose().getY(), swerve.getVelocityFieldRelative().vyMetersPerSecond);
            })
        .andThen(
            swerve.driveVelocityFieldRelative(
                () -> {
                  final var speeds =
                      new ChassisSpeeds(
                          vxController.calculate(swerve.getPose().getX(), cachedTarget[0].getX())
                              + vxController.getSetpoint().velocity,
                          vyController.calculate(swerve.getPose().getY(), cachedTarget[0].getY())
                              + vyController.getSetpoint().velocity,
                          headingController.calculate(
                                  swerve.getPose().getRotation().getRadians(),
                                  cachedTarget[0].getRotation().getRadians())
                              + headingController.getSetpoint().velocity);
                  Logger.recordOutput(
                      "AutoAim/Target Pose",
                      new Pose2d(
                          vxController.getSetpoint().position,
                          vyController.getSetpoint().position,
                          Rotation2d.fromRadians(headingController.getSetpoint().position)));
                  Logger.recordOutput("AutoAim/Target Speeds", speeds);
                  return speeds;
                }));
  }

  public static Command translateToReef(SwerveSubsystem swerve, Supplier<AutoAimTargets> target) {
    return translateToPose(swerve, () -> target.get().location);
  }
}
