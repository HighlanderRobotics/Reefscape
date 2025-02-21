package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.vision.Vision.VisionConstants;

public class KelpieSwerveConstants extends SwerveConstants {
  @Override
  public double getMaxLinearSpeed() {
    // From https://www.swervedrivespecialties.com/products/mk4n-swerve-module, L2+ with KrakenX60
    // and FOC
    return Units.feetToMeters(17.1);
  }

  @Override
  public double getMaxLinearAcceleration() {
    // Copied from Alpha
    return 14.0;
  }

  @Override
  public double getTrackWidthX() {
    return Units.inchesToMeters(23.75);
  }

  @Override
  public double getTrackWidthY() {
    return Units.inchesToMeters(23.75);
  }

  @Override
  public int getGyroID() {
    return 0;
  }

  @Override
  public double getHeadingVelocityKP() {
    // Copied from Alpha
    return 6.0;
  }

  @Override
  public double getHeadingVoltageKP() {
    // Copied from Alpha
    return 4.0;
  }

  @Override
  public Module.ModuleConstants getFrontLeftModule() {
    return new Module.ModuleConstants(0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(0.215576));
  }

  @Override
  public Module.ModuleConstants getFrontRightModule() {
    return new Module.ModuleConstants(
        1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(0.011719));
  }

  @Override
  public Module.ModuleConstants getBackLeftModule() {
    return new Module.ModuleConstants(2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(-0.276855));
  }

  @Override
  public Module.ModuleConstants getBackRightModule() {
    return new Module.ModuleConstants(
        3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(-0.211426));
  }

  @Override
  public AprilTagFieldLayout getFieldTagLayout() {
    return fieldTags;
  }

  @Override
  public double getDriveGearRatio() {
    // Taken from https://www.swervedrivespecialties.com/products/mk4n-swerve-module, L2+
    // configuration
    return (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
  }

  @Override
  public double getTurnGearRatio() {
    // For SDS Mk4n
    return 18.75;
  }

  @Override
  public TalonFXConfiguration getDriveConfig() {
    var driveConfig = new TalonFXConfiguration();
    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    driveConfig.Feedback.SensorToMechanismRatio = getDriveRotorToMeters();
    // Current control gains
    // Gains copied from AlphaSwerveConstants
    driveConfig.Slot0.kV = 5.0;
    // kT (stall torque / stall current) converted to linear wheel frame
    driveConfig.Slot0.kA = 0.0; // (9.37 / 483.0) / getDriveRotorToMeters(); // 3.07135116146;
    driveConfig.Slot0.kS = 10.0;
    driveConfig.Slot0.kP = 300.0;
    driveConfig.Slot0.kD = 0.0; // 1.0;

    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = getMaxLinearSpeed();
    driveConfig.MotionMagic.MotionMagicAcceleration = getMaxLinearAcceleration();

    return driveConfig;
  }

  @Override
  public TalonFXConfiguration getTurnConfig(int cancoderID) {
    // Copied from AlphaSwerveConstants
    var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        getTurnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Fused Cancoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    turnConfig.Feedback.RotorToSensorRatio = getTurnGearRatio();
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset = 0.0;
    // Controls Gains
    turnConfig.Slot0.kV = 0.42962962963; // ((5800 / 60) / getTurnGearRatio()) / 12
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.27;
    turnConfig.Slot0.kP = 20.0;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / getTurnGearRatio();
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (getTurnGearRatio() * 0.020);
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return turnConfig;
  }

  @Override
  public VisionConstants[] getVisionConstants() {
    final Matrix<N3, N3> BACK_LEFT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 906.46, 0.0, 675.30, 0.0, 907.49, 394.45, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> BACK_LEFT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.039, -0.057, -0.005, 0.001, -0.004, -0.001, 0.003, 0.001);
    final Matrix<N3, N3> BACK_RIGHT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 925.82, 0.0, 633.65, 0.0, 927.87, 386.90, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> BACK_RIGHT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.058, -0.09, 0.006, -0.003, 0.022, -0.002, 0.004, -0.001);
    final Matrix<N3, N3> FRONT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(), 911.67, 0.0, 663.03, 0.0, 909.82, 408.72, 0.0, 0.0, 1.0);
    final Matrix<N8, N1> FRONT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(), Nat.N1(), 0.044, -0.069, 0.001, 0.001, 0.013, -0.002, 0.004, 0.001);
    final VisionConstants backLeftCamConstants =
        new VisionConstants(
            "Back_Left",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.600),
                    Units.inchesToMeters(11.400),
                    Units.inchesToMeters(9.052)),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-28.125),
                    Units.degreesToRadians(150))),
            BACK_LEFT_CAMERA_MATRIX,
            BACK_LEFT_DIST_COEFFS);
    final VisionConstants backRightCamConstants =
        new VisionConstants(
            "Back_Right",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.600),
                    Units.inchesToMeters(-11.400488),
                    Units.inchesToMeters(9.052)),
                new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(210))),
            BACK_RIGHT_CAMERA_MATRIX,
            BACK_RIGHT_DIST_COEFFS);
    final VisionConstants frontCamConstants =
        new VisionConstants(
            "Front_Camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(11.879),
                    Units.inchesToMeters(-11.664),
                    Units.inchesToMeters(8.804)),
                new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(30))),
            FRONT_CAMERA_MATRIX,
            FRONT_DIST_COEFFS);
    return new VisionConstants[] {frontCamConstants, backLeftCamConstants, backRightCamConstants};
  }

  @Override
  public CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset) {
    // Copied from Alpha
    final var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        getTurnMotorInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    return cancoderConfig;
  }

  @Override
  public Mass getMass() {
    // Some materials not defined
    // Not final: Funnel and climber not complete, I've just set it to max robot weight
    return Pound.of(148);
  }

  @Override
  public double getBumperWidth() {
    return Units.inchesToMeters(36.6);
  }

  @Override
  public double getBumperLength() {
    return Units.inchesToMeters(36.6);
  }
}
