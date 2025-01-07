// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final AutoFactory factory;

  public Autos(SwerveSubsystem swerve) {
    this.swerve = swerve;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
            });
  }

  public AutoFactory getFactory() {
    return factory;
  }

  public Command getNoneAuto() {
    final var routine = factory.newRoutine("None");
    routine.active().onTrue(Commands.print("Running empty auto."));
    return routine.cmd();
  }

  public Command getTestTriangle() {
    final var routine = factory.newRoutine("Test Triangle");
    final var traj = routine.trajectory("Triangle Test");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    return routine.cmd();
  }

  public Command getTestSprint() {
    var routine = factory.newRoutine("Test Sprint");
    var traj = routine.trajectory("Sprint Test");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    return routine.cmd();
  }

  public Command getDCycle() {
    final var routine = factory.newRoutine("Cycle RHS Start to D");
    final var startToD = routine.trajectory("RHStoD");
    final var DtoPRO = routine.trajectory("DtoPRO");
    final var PROtoD = routine.trajectory("PROtoD");

    routine.active().whileTrue(Commands.sequence(startToD.resetOdometry(), startToD.cmd()));
    routine
        .observe(startToD.done())
        .onTrue(
            Commands.sequence(
                // score
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> DtoPRO.getInitialPose().orElse(Pose2d.kZero))),
                DtoPRO.cmd()));
    routine
        .observe(DtoPRO.done())
        .onTrue(
            Commands.sequence(
                // intake
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> PROtoD.getInitialPose().orElse(Pose2d.kZero))),
                PROtoD.cmd()));
    routine
        .observe(PROtoD.done())
        .onTrue(
            Commands.sequence(
                // score
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> DtoPRO.getInitialPose().orElse(Pose2d.kZero))),
                PROtoD.cmd()));

    return routine.cmd();
  }
}
