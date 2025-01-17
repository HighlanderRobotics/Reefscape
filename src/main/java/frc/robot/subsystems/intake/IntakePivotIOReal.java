package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class IntakePivotIOReal implements IntakePivotIO {
  // TODO: SET DEVICE ID
  private final TalonFX pivotMotor = new TalonFX(10, "*");

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec = pivotMotor.getVelocity();
  private final StatusSignal<Temperature> temp = pivotMotor.getDeviceTemp();
  private final StatusSignal<Current> supplyCurrentAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = pivotMotor.getStatorCurrent();
  private final StatusSignal<Angle> motorPositionRotations = pivotMotor.getPosition();

  public IntakePivotIOReal() {
    // TODO: MOTOR CONFIGURATION
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec, temp, supplyCurrentAmps, statorCurrentAmps, motorPositionRotations);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.tempDegreesC = temp.getValue().in(Units.Celsius);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.pivotPosition = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
  }

  @Override
  public void setMotorVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    pivotMotor.setPosition(targetPosition.getRotations());
  }
}
