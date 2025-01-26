package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  public static final Rotation2d LATCH_CLOSED_POSITION = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d LATCH_OPEN_POSITION = Rotation2d.fromDegrees(180.0);
  private final RollerIO leftIO, rightIO;
  private final RollerIOInputsAutoLogged leftInputs = new RollerIOInputsAutoLogged(),
      rightInputs = new RollerIOInputsAutoLogged();
  // this could be in its own subsystem but it doesnt really matter tbh
  private final ServoIO latchIO;
  private final ServoIOInputsAutoLogged latchInputs = new ServoIOInputsAutoLogged();

  public FunnelSubsystem(RollerIO leftIO, RollerIO rightIO, ServoIO latchIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;
    this.latchIO = latchIO;

    latchIO.setPosition(LATCH_CLOSED_POSITION);
  }

  @Override
  public void periodic() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);
    // no updating latch inputs bc blank inputs
    Logger.processInputs("Funnel/Left", leftInputs);
    Logger.processInputs("Funnel/Right", rightInputs);
    Logger.processInputs("Funnel/Latch", latchInputs);
  }

  public Command runVoltage(final double volts) {
    return this.run(
        () -> {
          leftIO.setVoltage(volts);
          rightIO.setVoltage(volts);
        });
  }

  /** DO NOT RUN IF YOU WANT TO SCORE MORE CORAL */
  public Command unlatch() {
    return this.runOnce(() -> latchIO.setPosition(LATCH_OPEN_POSITION));
  }
}
