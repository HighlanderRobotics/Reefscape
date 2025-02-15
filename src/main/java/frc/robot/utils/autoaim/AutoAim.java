package frc.robot.utils.autoaim;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAim {
  static final double MAX_ANGULAR_SPEED = 10.0;
  static final double MAX_ANGULAR_ACCELERATION = 10.0;
  static final double MAX_AUTOAIM_SPEED = 3.0;
  static final double MAX_AUTOAIM_ACCELERATION = 4.0;

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  public static final Translation2d RED_REEF_CENTER = ChoreoAllianceFlipUtil.flip(BLUE_REEF_CENTER);

  public static double BLUE_NET_X = 8.08;
  public static double RED_NET_X = ChoreoAllianceFlipUtil.flipX(BLUE_NET_X);

  public static final double TRANSLATION_TOLERANCE_METERS = Units.inchesToMeters(2.0);
  public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);

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
            10.0,
            0.01,
            0.0,
            new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    final ProfiledPIDController vyController =
        new ProfiledPIDController(
            10.0,
            0.01,
            0.0,
            new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    return Commands.runOnce(
            () -> {
              cachedTarget[0] = target.get();
              final var diff = swerve.getPose().minus(cachedTarget[0]);
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
                  final var diff = swerve.getPose().minus(cachedTarget[0]);
                  final var speeds =
                      MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.25))
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.25))
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                          ? new ChassisSpeeds()
                          : new ChassisSpeeds(
                              vxController.calculate(
                                      swerve.getPose().getX(), cachedTarget[0].getX())
                                  + vxController.getSetpoint().velocity,
                              vyController.calculate(
                                      swerve.getPose().getY(), cachedTarget[0].getY())
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

  public static Command translateToXCoord(
      SwerveSubsystem swerve,
      DoubleSupplier x,
      DoubleSupplier yVel,
      Supplier<Rotation2d> headingTarget) {
    // This feels like a horrible way of getting around lambda final requirements
    // Is there a cleaner way of doing this?
    // The y of this isn't used
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
            6.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
    return Commands.runOnce(
            () -> {
              cachedTarget[0] = new Pose2d(x.getAsDouble(), 0, headingTarget.get());
              Logger.recordOutput("AutoAim/Cached Target", cachedTarget[0]);
              headingController.reset(swerve.getPose().getRotation().getRadians(), 0.0);
              vxController.reset(swerve.getPose().getX(), 0.0);
            })
        .andThen(
            swerve.driveVelocityFieldRelative(
                () -> {
                  final var diff = swerve.getPose().minus(cachedTarget[0]);
                  final var speeds =
                      MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.25))
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.25))
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                          ? new ChassisSpeeds()
                          : new ChassisSpeeds(
                              vxController.calculate(
                                      swerve.getPose().getX(), cachedTarget[0].getX())
                                  + vxController.getSetpoint().velocity,
                              // Use the inputted y velocity target
                              yVel.getAsDouble(),
                              headingController.calculate(
                                      swerve.getPose().getRotation().getRadians(),
                                      cachedTarget[0].getRotation().getRadians())
                                  + headingController.getSetpoint().velocity);
                  Logger.recordOutput(
                      "AutoAim/Target Pose",
                      new Pose2d(
                          vxController.getSetpoint().position,
                          0,
                          Rotation2d.fromRadians(headingController.getSetpoint().position)));
                  Logger.recordOutput("AutoAim/Target Speeds", speeds);
                  return speeds;
                }));
  }

  public static boolean isInToleranceCoral(Pose2d pose) {
    final var diff = pose.minus(CoralTargets.getClosestTarget(pose));
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAim.TRANSLATION_TOLERANCE_METERS)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAim.ROTATION_TOLERANCE_RADIANS);
  }

  public static boolean isInToleranceAlgaeIntake(Pose2d pose) {
    final var diff = pose.minus(AlgaeIntakeTargets.getClosestTarget(pose));
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAim.TRANSLATION_TOLERANCE_METERS)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAim.ROTATION_TOLERANCE_RADIANS);
  }

  public static boolean isInTolerance(Pose2d pose, Pose2d pose2) {
    final var diff = pose.minus(pose2);
    return MathUtil.isNear(
            0.0, Math.hypot(diff.getX(), diff.getY()), AutoAim.TRANSLATION_TOLERANCE_METERS)
        && MathUtil.isNear(
            0.0, diff.getRotation().getRadians(), AutoAim.ROTATION_TOLERANCE_RADIANS);
  }
}
