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
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.AutoAimTargets;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final ManipulatorSubsystem manipulator;
  private final AutoFactory factory;
  private final ElevatorSubsystem elevator;

  public Autos(
      SwerveSubsystem swerve, ManipulatorSubsystem manipulator, ElevatorSubsystem elevator) {
    this.swerve = swerve;
    this.manipulator = manipulator;
    this.elevator = elevator;
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

  /***
   * Waits for the current path to be completed then runs the command (score or intake), waits to be close enough then starts the next path
   * //TODO order does not make sense (?)
   * @param routine AutoRoutine these paths are in
   * @param currentPath The path that just completed
   * @param nextPath The path that is about to run
   * @param cmd The command we want to run before nextPath
   */
  public void runPath(
      AutoRoutine routine, AutoTrajectory currentPath, AutoTrajectory nextPath, Command cmd) {
    routine
        .observe(currentPath.done())
        .onTrue(
            Commands.sequence(
                cmd,
                Commands.waitUntil(
                        () -> {
                          final var diff =
                              swerve
                                  .getPose()
                                  .minus(nextPath.getInitialPose().orElse(Pose2d.kZero));
                          return MathUtil.isNear(
                                  0.0,
                                  diff.getX(),
                                  Units.inchesToMeters(2.0)) // TODO update when merged
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(2.0))
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
                        })
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> nextPath.getInitialPose().orElse(Pose2d.kZero))),
                nextPath.cmd()));
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

  /***
   * Runs a path based on the names of the starting location, middle location, and ending location. The current path is generated based on starting + middle and the next from middle + end
   * @param routine AutoRoutine these paths are in
   * @param startLocation Starting location of the path that just completed
   * @param middleLocation Ending location of the path that just completed/starting location of the next one
   * @param endLocation Ending location of the next path
   * @param steps Relevant AutoTrajectories + names
   */
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
                    traj.cmd()
                        .raceWith(
                            Commands.waitUntil(
                                    () -> {
                                      final var diff =
                                          swerve
                                              .getPose()
                                              .minus(traj.getFinalPose().orElse(Pose2d.kZero));
                                      return MathUtil.isNear(
                                              0.0,
                                              diff.getX(),
                                              Units.inchesToMeters(15.0)) // TODO find tolerances
                                          && MathUtil.isNear(
                                              0.0, diff.getY(), Units.inchesToMeters(15.0));
                                    })
                                .andThen(
                                    elevator
                                        .setExtension(ElevatorSubsystem.L4_EXTENSION_METERS)
                                        .asProxy())))
                .raceWith(manipulator.backIndex()));
    routine.observe(traj.done()).onTrue(scoreInAuto(traj.getFinalPose(), ReefTarget.L4));
    return routine.cmd();
  }

  public Command debugAuto() {
    // final var routine = factory.newRoutine("LM to H");
    // final var traj = routine.trajectory("LMtoH");
    // routine
    //     .active()
    //     .whileTrue(
    //         Commands.sequence(
    //             traj.resetOdometry(),
    //             traj.cmd(),
    return scoreInAuto(
            Optional.of(AutoAimTargets.getRobotTargetLocation(AutoAimTargets.RED_L.location)),
            ReefTarget.L4)
        .asProxy()
        .alongWith(elevator.setExtension(ElevatorSubsystem.L4_EXTENSION_METERS).asProxy());
    // return routine.cmd();
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
                swerve.poseLockDriveCommand(() -> traj.getFinalPose().orElse(Pose2d.kZero)),
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

  public Command LItoKCMD() {
    final var routine = factory.newRoutine("LI to K");
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "LI", "K", "PLI", "L", "PLI", "A", "PLI", "B", "PLI" // each stop we are going to, in order
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
                steps.get("LItoK").resetOdometry(),
                steps.get("LItoK").cmd())); // runs the very first traj
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

  public Command RItoDCMD() {
    final var routine = factory.newRoutine("RI to D");
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "RI", "D", "PRI", "C", "PRI", "B", "PRI", "A", "PRI" // each stop we are going to, in order
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
                steps.get("RItoD").resetOdometry(),
                steps.get("RItoD").cmd())); // runs the very first traj
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
      return Commands.none();
    } else {
      return AutoAim.translateToPose(swerve, () -> pose.get())
          .until(
              () -> {
                final var diff = swerve.getPose().minus(pose.get());
                return MathUtil.isNear(
                        0.0, diff.getX(), Units.inchesToMeters(2.0)) // TODO find tolerances
                    && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(2.0))
                    && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
              })
          .andThen(
              Commands.race(
                  Commands.waitUntil(() -> !manipulator.getSecondBeambreak()),
                  manipulator.setVelocity(
                      10.0))); // TODO this needs to be changed to match the enum later but i'm on
      // the wrong branch
    }
  }

  public Command intakeInAuto(Optional<Pose2d> pose) {
    if (!pose.isPresent()) {
      return Commands.none();
    } else {
      return AutoAim.translateToPose(swerve, () -> pose.get())
          .until(() -> manipulator.getSecondBeambreak() || Robot.isSimulation());
    }
  }
}
