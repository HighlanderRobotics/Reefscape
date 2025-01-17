package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;

public class IntakeRollerSubsystem extends RollerSubsystem {
    // TODO: SET TO GOOD VALUE
    static final double INTAKE_VELOCITY = 10.0;

    public IntakeRollerSubsystem(RollerIO io) {
        super(io, "Intake/Roller");
    }

    public Command intake() {
        // This 2 secs is arbitrary
        return setVelocity(INTAKE_VELOCITY).withTimeout(2);
    }

    public Command intake(Time timeout) {
        // This 2 secs is arbitrary
        return setVelocity(INTAKE_VELOCITY).withTimeout(timeout);
    }

}
