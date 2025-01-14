package frc.robot.utils.autoaim;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAim {
  static double MAX_ANGULAR_SPEED = 1.0;
  static double MAX_ANGULAR_ACCELERATION = 1.0;
  static double MAX_AUTOAIM_SPEED = 1.0;
  static double MAX_AUTOAIM_ACCELERATION = 1.0;

  public static final Command translateToReef(SwerveSubsystem swerve, Pose2d target) {
    ProfiledPIDController headingController =
        // assume we can accelerate to max in 2/3 of a second
        new ProfiledPIDController(
            0.5,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED / 2, MAX_ANGULAR_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    ProfiledPIDController vxController =
        new ProfiledPIDController(
            0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    ProfiledPIDController vyController =
        new ProfiledPIDController(
            0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    return Commands.sequence(
            swerve.driveVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        vxController.calculate(swerve.getPose().getX(), target.getX())
                            + vxController.getSetpoint().velocity,
                        vyController.calculate(swerve.getPose().getY(), target.getY())
                            + vyController.getSetpoint().velocity,
                        headingController.calculate(
                                swerve.getPose().getRotation().getRadians(),
                                target.getRotation().getRadians())
                            + headingController.getSetpoint().velocity)))
        .beforeStarting(
            () -> {
              vxController.setConstraints(
                  new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_AUTOAIM_SPEED));
              vyController.setConstraints(
                  new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_AUTOAIM_SPEED));
              headingController.reset(
                  new TrapezoidProfile.State(
                      swerve.getPose().getRotation().getRadians(),
                      swerve.getVelocity().omegaRadiansPerSecond));
              vxController.reset(
                  new TrapezoidProfile.State(
                      swerve.getPose().getX(), swerve.getVelocity().vxMetersPerSecond));
              vyController.reset(
                  new TrapezoidProfile.State(
                      swerve.getPose().getY(), swerve.getVelocity().vyMetersPerSecond));
            });
  }

  public static Command translateToReef(SwerveSubsystem swerve, AutoAimTargets target) {
    return translateToReef(swerve, target.location);
  }
}
