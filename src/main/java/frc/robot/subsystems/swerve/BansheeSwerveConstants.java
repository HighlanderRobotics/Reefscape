package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilogram;

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
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.Vision.VisionConstants;

public class BansheeSwerveConstants extends SwerveConstants {

  @Override
  public double getMaxLinearSpeed() {
    return Units.feetToMeters(16.0);
  }

  @Override
  public double getMaxLinearAcceleration() {
    return 8.0;
  }

  @Override
  public double getTrackWidthX() {
    return Units.inchesToMeters(21.75);
  }

  @Override
  public double getTrackWidthY() {
    return Units.inchesToMeters(21.25);
  }

  @Override
  public int getGyroID() {
    return 0;
  }

  @Override
  public double getHeadingVelocityKP() {
    return 4.0;
  }

  @Override
  public double getHeadingVoltageKP() {
    return 4.0;
  }

  @Override
  public ModuleConstants getFrontLeftModule() {
    return new ModuleConstants(0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(0.377930));
  }

  @Override
  public ModuleConstants getFrontRightModule() {
    return new ModuleConstants(1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(-0.071289));
  }

  @Override
  public ModuleConstants getBackLeftModule() {
    return new ModuleConstants(2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(0.550781));
  }

  @Override
  public ModuleConstants getBackRightModule() {
    return new ModuleConstants(3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(-0.481689));
  }

  // Need this annotation so the alert doesn't get mad
  @SuppressWarnings("resource")
  @Override
  public AprilTagFieldLayout getFieldTagLayout() {
    return fieldTags;
  }

  @Override
  public VisionConstants[] getVisionConstants() {
    final Matrix<N3, N3> LEFT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            915.2126592056358,
            0.0,
            841.560216921862,
            0.0,
            913.9556728013187,
            648.2330358379004,
            0.0,
            0.0,
            1.0);
    final Matrix<N8, N1> LEFT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.0576413369828492,
            -0.07356597379196807,
            -6.669129885790735E-4,
            6.491281122640802E-4,
            0.03731824873787814,
            0,
            0,
            0);
    final Matrix<N3, N3> RIGHT_CAMERA_MATRIX =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            902.0832829888818,
            0.0,
            611.9702186077134,
            0.0,
            902.2731968281233,
            400.755534902121,
            0.0,
            0.0,
            1.0);
    final Matrix<N8, N1> RIGHT_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.05398335403070431,
            -0.07589158973947994,
            -0.003081304772847505,
            -0.0010797674400397023,
            0.015185486932866137,
            0,
            0,
            0);
    final VisionConstants leftCamConstants =
        new VisionConstants(
            "Left_Camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.386),
                    Units.inchesToMeters(10.380),
                    Units.inchesToMeters(7.381)),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-28.125),
                    Units.degreesToRadians(120))),
            LEFT_CAMERA_MATRIX,
            LEFT_DIST_COEFFS);
    final VisionConstants rightCamConstants =
        new VisionConstants(
            "Right_Camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.597),
                    Units.inchesToMeters(-10.143),
                    Units.inchesToMeters(7.384)),
                new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(210))),
            RIGHT_CAMERA_MATRIX,
            RIGHT_DIST_COEFFS);
    return new VisionConstants[] {leftCamConstants, rightCamConstants};
  }

  @Override
  public double getDriveGearRatio() {
    return (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  }

  @Override
  public TalonFXConfiguration getDriveConfig() {
    final var driveConfig = new TalonFXConfiguration();
    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
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
    driveConfig.Slot0.kV = 0.0;
    // kT (stall torque / stall current) converted to linear wheel frame
    driveConfig.Slot0.kA = 0.0; // (9.37 / 483.0) / getDriveRotorToMeters(); // 3.07135116146;
    driveConfig.Slot0.kS = 14.0;
    driveConfig.Slot0.kP = 100.0;
    driveConfig.Slot0.kD = 1.0;

    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = getMaxLinearSpeed();
    driveConfig.MotionMagic.MotionMagicAcceleration = getMaxLinearAcceleration();
    return driveConfig;
  }

  @Override
  public TalonFXConfiguration getTurnConfig(int cancoderID) {
    final var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
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
    turnConfig.Slot0.kV = 2.7935;
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.28;
    turnConfig.Slot0.kP = 20.0;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / getTurnGearRatio();
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (getTurnGearRatio() * 0.1);
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    return turnConfig;
  }

  @Override
  public CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset) {
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
    return Kilogram.of(74.0);
  }

  @Override
  public double getBumperWidth() {
    return Units.inchesToMeters(35.16);
  }

  @Override
  public double getBumperLength() {
    return Units.inchesToMeters(39.66);
  }

  @Override
  public VisionConstants getAlgaeVisionConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAlgaeVisionConstants'");
  }
}
