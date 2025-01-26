package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX motor = new TalonFX(18, "*");

  private final StatusSignal<AngularVelocity> angularVelocityRPS = motor.getVelocity();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Angle> position = motor.getPosition();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public ClimberIOReal() {
    final var config = new TalonFXConfiguration();

    motor.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, angularVelocityRPS, temp, supplyCurrentAmps, statorCurrentAmps, position);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(angularVelocityRPS, temp, supplyCurrentAmps, statorCurrentAmps, position);

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.tempDegreesC = temp.getValue().in(Units.Celsius);
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.angularVelocityRPS = angularVelocityRPS.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setRotation(Rotation2d rotation) {
    motor.setControl(motionMagic.withPosition(rotation.getRotations()));
  }
}
