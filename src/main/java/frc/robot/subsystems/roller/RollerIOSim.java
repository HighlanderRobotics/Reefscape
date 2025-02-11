package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;
import java.util.function.Consumer;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforward;
  private final ProfiledPIDController pid;

  private Optional<Consumer<RollerIOInputsAutoLogged>> callback = Optional.empty();

  private double appliedVolts = 0.0;

  public RollerIOSim(
      double jKgMetersSquared,
      double gearRatio,
      SimpleMotorFeedforward feedforward,
      ProfiledPIDController pid) {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), jKgMetersSquared, gearRatio),
            DCMotor.getKrakenX60Foc(1));
    this.feedforward = feedforward;
    this.pid = pid;
  }

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    motorSim.update(0.02);
    inputs.velocityRotationsPerSec = motorSim.getAngularVelocityRPM() / 60;
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.statorCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.tempCelsius = 0.0;

    callback.ifPresent((cb) -> cb.accept(inputs));
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    motorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setVelocity(double velocityRPS) {
    setVoltage(
        feedforward.calculate(velocityRPS)
            + pid.calculate(motorSim.getAngularVelocityRPM() / 60, velocityRPS));
  }

  @Override
  public void registerSimulationCallback(Consumer<RollerIOInputsAutoLogged> callback) {
    this.callback = Optional.of(callback);
  }
}
