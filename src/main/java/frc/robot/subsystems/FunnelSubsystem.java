package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends RollerSubsystem {
  public static final Rotation2d LATCH_CLOSED_POSITION = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d LATCH_OPEN_POSITION = Rotation2d.fromDegrees(180.0);
  // this could be in its own subsystem but it doesnt really matter tbh
  private final ServoIO latchIO;
  private final ServoIOInputsAutoLogged latchInputs = new ServoIOInputsAutoLogged();

  public FunnelSubsystem(RollerIO io, ServoIO latchIO) {
    super(io, "Funnel");
    this.latchIO = latchIO;

    latchIO.setPosition(LATCH_CLOSED_POSITION);
  }

  @Override
  public void periodic() {
    super.periodic();
    // no updating latch inputs bc blank inputs
    Logger.processInputs("Funnel/Latch", latchInputs);
  }

  /** DO NOT RUN IF YOU WANT TO SCORE MORE CORAL */
  public Command unlatch() {
    return this.runOnce(() -> latchIO.setPosition(LATCH_OPEN_POSITION));
  }
}
