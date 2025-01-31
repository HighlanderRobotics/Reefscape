package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX motor = new TalonFX(18, "*");

  private final StatusSignal<AngularVelocity> angularVelocityRPS = motor.getVelocity();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
  private final StatusSignal<Angle> position = motor.getPosition();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);

  public ClimberIOReal() {
    final var config = new TalonFXConfiguration();

    config.Slot0.kP = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 5.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 0.75;

    config.Feedback.SensorToMechanismRatio = ClimberSubsystem.CLIMB_GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, angularVelocityRPS, temp, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRPS, temp, supplyCurrentAmps, statorCurrentAmps, position, appliedVoltage);

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.tempDegreesC = temp.getValue().in(Units.Celsius);
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.angularVelocityRPS = angularVelocityRPS.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(final double position) {
    motor.setControl(motionMagic.withPosition(position));
  }
}
