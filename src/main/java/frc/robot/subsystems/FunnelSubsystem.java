package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends RollerSubsystem {
  public static final Rotation2d FIRST_LATCH_CLOSED_POSITION = Rotation2d.fromDegrees(125.0);
  public static final Rotation2d FIRST_LATCH_OPEN_POSITION = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d SECOND_LATCH_CLOSED_POSITION = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d SECOND_LATCH_OPEN_POSITION = Rotation2d.fromDegrees(125.0);
  // this could be in its own subsystem but it doesnt really matter tbh
  private final ServoIO firstLatchIO, secondLatchIO;
  private final ServoIOInputsAutoLogged firstLatchInputs = new ServoIOInputsAutoLogged(),
      secondLatchInputs = new ServoIOInputsAutoLogged();

  public FunnelSubsystem(RollerIO io, ServoIO firstLatchIO, ServoIO secondLatchIO) {
    super(io, "Funnel");
    this.firstLatchIO = firstLatchIO;
    this.secondLatchIO = secondLatchIO;

    firstLatchIO.setPosition(FIRST_LATCH_CLOSED_POSITION);
    secondLatchIO.setPosition(SECOND_LATCH_CLOSED_POSITION);
  }

  @Override
  public void periodic() {
    super.periodic();
    // no updating latch inputs bc blank inputs
    Logger.processInputs("Funnel/First Latch", firstLatchInputs);
    Logger.processInputs("Funnel/Second Latch", secondLatchInputs);
  }

  /** DO NOT RUN IF YOU WANT TO SCORE MORE CORAL */
  public Command unlatch() {
    return this.runOnce(
        () -> {
          firstLatchIO.setPosition(FIRST_LATCH_OPEN_POSITION);
          secondLatchIO.setPosition(SECOND_LATCH_OPEN_POSITION);
        });
  }
}
