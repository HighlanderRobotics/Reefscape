package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class IntakePivotIOReal implements IntakePivotIO {
  // TODO: SET DEVICE ID
  private final TalonFX motor = new TalonFX(10, "*");

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec = motor.getVelocity();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Angle> motorPositionRotations = motor.getPosition();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public IntakePivotIOReal() {
    // TODO: MOTOR CONFIGURATION
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // Gains currently from the sim. Need to be updated with our own robot's
    config.Slot0.kV = 0.543;
    config.Slot0.kG = 0.3856;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 5.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 2.6;

    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 1.0;

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, angularVelocityRotsPerSec, temp, supplyCurrentAmps, statorCurrentAmps, motorPositionRotations);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        temp,
        supplyCurrentAmps,
        statorCurrentAmps,
        motorPositionRotations);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.tempDegreesC = temp.getValue().in(Units.Celsius);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
  }
}
