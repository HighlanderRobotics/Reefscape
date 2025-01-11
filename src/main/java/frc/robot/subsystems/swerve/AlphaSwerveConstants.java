package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.Pound;

// TODO: Set constants!
public class AlphaSwerveConstants extends SwerveConstants {
    @Override
    public double getMaxLinearSpeed() {
        return 0;
    }

    @Override
    public double getMaxLinearAcceleration() {
        return 0;
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
        return 0;
    }

    @Override
    public double getHeadingVoltageKP() {
        return 0;
    }

    @Override
    public Module.ModuleConstants getFrontLeftModule() {
        return null;
    }

    @Override
    public Module.ModuleConstants getFrontRightModule() {
        return null;
    }

    @Override
    public Module.ModuleConstants getBackLeftModule() {
        return null;
    }

    @Override
    public Module.ModuleConstants getBackRightModule() {
        return null;
    }

    @Override
    public AprilTagFieldLayout getFieldTagLayout() {
        return fieldTags;
    }

    @Override
    public double getDriveGearRatio() {
        // Taken from https://www.swervedrivespecialties.com/products/mk4n-swerve-module, L2+ configuration
        return (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    }

    @Override
    public TalonFXConfiguration getDriveConfig() {
        // Copied from Banshee w/ control gains set to 0.0
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
        // TODO: Set gains
        driveConfig.Slot0.kV = 0.0;
        // kT (stall torque / stall current) converted to linear wheel frame
        driveConfig.Slot0.kA = 0.0;
        driveConfig.Slot0.kS = 0.0;
        driveConfig.Slot0.kP = 0.0;
        driveConfig.Slot0.kD = 0.0;

        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

        driveConfig.MotionMagic.MotionMagicCruiseVelocity = getMaxLinearSpeed();
        driveConfig.MotionMagic.MotionMagicAcceleration = getMaxLinearAcceleration();
        return driveConfig;
    }

    @Override
    public TalonFXConfiguration getTurnConfig(int cancoderID) {
        // Copied from Banshee w/ control gains set to 0.0
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
        // TODO: Set gains
        turnConfig.Slot0.kV = 0.0;
        turnConfig.Slot0.kA = 0.0;
        turnConfig.Slot0.kS = 0.0;
        turnConfig.Slot0.kP = 0.0;
        turnConfig.Slot0.kD = 0.0;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / getTurnGearRatio();
        turnConfig.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (getTurnGearRatio() * 0.1);
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        return turnConfig;
    }

    @Override
    public CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset) {
        // Copied from Banshee
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
        // Onshape says some subsystems have no material. May need to be updated
        return Pound.of(99.8 + 13);
    }
}
