package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
  // TODO: SET TO ACTUAL RATIO WHEN CAD IS FINISHED
  public static double PIVOT_RATIO = 1.0;

  private IntakePivotIO io;
  private IntakePivotIOInputsAutoLogged inputs;

  public IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);
  }

  public Command setTargetAngle(Rotation2d target) {
    return Commands.runOnce(() -> Logger.recordOutput("Intake/PivotSetpoint", target))
        .andThen(() -> io.setMotorPosition(target));
  }
}
