package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;

public class IntakeRollerSubsystem extends RollerSubsystem {
  // TODO: SET TO GOOD VALUE
  static final double INTAKE_VELOCITY = 10.0;

  public IntakeRollerSubsystem(RollerIO io) {
    super(io, "Intake/Roller");
  }

  // Runs intake
  public Command intake() {
    return intake(INTAKE_VELOCITY);
  }

  // Runs intake at specified velocity
  public Command intake(double velocity) {
    return setVelocity(velocity);
  }

  // Runs intake in reverse
  public Command outtake() {
    return intake(-INTAKE_VELOCITY);
  }

  // Runs intake in reverse at the specified velocity
  public Command outtake(double velocity) {
    return intake(-velocity);
  }
}
