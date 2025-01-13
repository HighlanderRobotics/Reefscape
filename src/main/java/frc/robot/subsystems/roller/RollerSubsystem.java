// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private final String name;
  public static final double INDEXING_VELOCITY = 40; // TODO

  /**
   * Creates a new RollerSubsystem.
   *
   * @param io an io impl for this subsystem to use. Should be RollerIOReal in all cases, use the
   *     callback to handle sim behavior.
   * @param name a name to be used for logging this subsystem. Should be unique.
   */
  public RollerSubsystem(RollerIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command setVelocity(DoubleSupplier vel) {
    return this.run(() -> io.setVelocity(vel.getAsDouble()));
  }

  public Command setVelocity(double vel) {
    return this.setVelocity(() -> vel);
  }
}
