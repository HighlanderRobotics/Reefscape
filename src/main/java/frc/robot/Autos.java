// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final ManipulatorSubsystem manipulator;
  private final AutoFactory factory;

  public Autos(SwerveSubsystem swerve, ManipulatorSubsystem manipulator) {
    this.swerve = swerve;
    this.manipulator = manipulator;
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

  public Command getPROtoD() {
    var routine = factory.newRoutine("coral intake to D");
    var traj = routine.trajectory("PROtoD");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    return routine.cmd();
  }

  public Command getStarttoD() {
    var routine = factory.newRoutine("start postion to D");
    var traj = routine.trajectory("StarttoD");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    return routine.cmd();
  }

  public Command autoCycleTest() {
    final var routine = factory.newRoutine("Cycle RHS Start to D");
    final var DtoPRO = routine.trajectory("DtoPRO");
    final var PROtoD = routine.trajectory("PROtoD");
    final var StarttoD = routine.trajectory("StarttoD");

    routine.active().whileTrue(Commands.sequence(StarttoD.resetOdometry(), StarttoD.cmd()));
    routine
        .observe(StarttoD.done())
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
                // getcoral
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
                DtoPRO.cmd()));

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
                PROtoD.cmd())); // should be DtoPRO instead?

    return routine.cmd();
  }

  public void runPath(
      AutoRoutine routine, AutoTrajectory currentPath, AutoTrajectory nextPath, Command cmd) {
    routine
        .observe(currentPath.done())
        .onTrue(
            Commands.sequence(
                cmd,
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> nextPath.getInitialPose().orElse(Pose2d.kZero))),
                nextPath.cmd()));
    routine.cmd();
  }

  // just runs a single path
  public void runPath(AutoRoutine routine, AutoTrajectory path, Command cmd) {
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                path.resetOdometry(),
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> path.getInitialPose().orElse(Pose2d.kZero))),
                path.cmd(),
                cmd));
  }

  public void runPath(
      AutoRoutine routine,
      String startLocation,
      String middleLocation,
      String endLocation,
      HashMap<String, AutoTrajectory> steps) {
    runPath(
        routine,
        steps.get(startLocation + "to" + middleLocation),
        steps.get(middleLocation + "to" + endLocation),
        endLocation.substring(0, 1).equals("P") // hp station
            ? intakeInAuto(steps.get(middleLocation + "to" + endLocation).getFinalPose())
            : scoreInAuto(
                steps.get(middleLocation + "to" + endLocation).getFinalPose(),
                ReefTarget.L4)); // i've been told we're only scoring on l4 in auto
  }

  public Command SLMtoICMD() {
    final var routine = factory.newRoutine("SLM to I");
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "SLM", "I", "PLO", "L", "PLO", "K", "PLO", "J" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                steps.get("SLMtoI").resetOdometry(),
                steps.get("SLMtoI").cmd())); // runs the very first traj
    for (int i = 0; i < stops.length - 2; i++) {
      runPath(
          routine,
          stops[i],
          stops[i + 1],
          stops[i + 2],
          steps); // runs each of the following traj + whatever it does at the end of the traj
    }
    return routine.cmd();
  }

  public Command LMtoHCMD() {
    final var routine = factory.newRoutine("LM to H");
    final var traj = routine.trajectory("LMtoH");
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                traj.resetOdometry(),
                traj.cmd(),
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> traj.getFinalPose().orElse(Pose2d.kZero))),
                scoreInAuto(traj.getFinalPose(), ReefTarget.L4)));
    return routine.cmd();
  }

  public Command RMtoGCmd() {
    final var routine = factory.newRoutine("RM to G");
    final var traj = routine.trajectory("RMtoG");
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                traj.resetOdometry(),
                traj.cmd(),
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> traj.getFinalPose().orElse(Pose2d.kZero))),
                scoreInAuto(traj.getFinalPose(), ReefTarget.L4)));
    return routine.cmd();
  }

  public Command LOtoJCMD() {
    final var routine = factory.newRoutine("LO to J");
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "LO", "J", "PLO", "K", "PLO", "L", "PLO", "A", "PLO" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                steps.get("LOtoJ").resetOdometry(),
                steps.get("LOtoJ").cmd())); // runs the very first traj
    for (int i = 0; i < stops.length - 2; i++) {
      runPath(
          routine,
          stops[i],
          stops[i + 1],
          stops[i + 2],
          steps); // runs each of the following traj + whatever it does at the end of the traj
    }
    return routine.cmd();
  }

  public Command ROtoECMD() {
    final var routine = factory.newRoutine("RO to E");
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "RO", "E", "PRO", "D", "PRO", "C", "PRO", "B", "PRO" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                steps.get("ROtoE").resetOdometry(),
                steps.get("ROtoE").cmd())); // runs the very first traj
    for (int i = 0; i < stops.length - 2; i++) {
      runPath(
          routine,
          stops[i],
          stops[i + 1],
          stops[i + 2],
          steps); // runs each of the following traj + whatever it does at the end of the traj
    }
    return routine.cmd();
  }

  public Command scoreInAuto(Optional<Pose2d> pose, ReefTarget target) {
    if (!pose.isPresent()) {
      return new InstantCommand();
    } else {
      return Commands.sequence(
          Commands.parallel(
              AutoAim.translateToPose(swerve, () -> pose.get()),
              Commands.waitUntil(
                  () -> {
                    final var diff = swerve.getPose().minus(pose.get());
                    return MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(1.0))
                        && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(1.0))
                        && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
                  }),
              Commands.race(
                      Commands.waitUntil(() -> !manipulator.getSecondBeambreak()),
                      manipulator.setVelocity(() -> target == ReefTarget.L1 ? 12.0 : 100.0))
                  .andThen(
                      Commands.waitSeconds(0.75),
                      Commands.waitUntil(
                          () -> {
                            final var diff = swerve.getPose().minus(pose.get());
                            return !(MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(6.0))
                                && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(6.0)));
                          }))));
    }
  }

  public Command intakeInAuto(Optional<Pose2d> pose) {
    if (!pose.isPresent()) {
      return new InstantCommand();
    } else {
      return Commands.sequence(
          Commands.parallel(
              AutoAim.translateToPose(swerve, () -> pose.get()),
              Commands.waitUntil(
                  () -> {
                    final var diff = swerve.getPose().minus(pose.get());
                    return MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(1.0))
                        && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(1.0))
                        && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
                  }),
              Commands.waitUntil(() -> manipulator.getSecondBeambreak())));
    }
  }

 

}
