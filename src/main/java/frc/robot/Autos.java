// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final ManipulatorSubsystem manipulator;
  private final AutoFactory factory;
  private final ElevatorSubsystem elevator;

  public static boolean autoPreScore = false;
  public static boolean autoScore = false; // TODO perhaps this should not be static

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
   * @param prevPath The path that just completed
   * @param currentPath The path that is about to run
   * @param cmd The command we want to run before nextPath
   */
  public void cmdThenPath(
      AutoRoutine routine, AutoTrajectory prevPath, AutoTrajectory currentPath, Command cmd) {
    routine
        .observe(prevPath.done())
        .onTrue(
            Commands.sequence(
                cmd,
                Commands.waitUntil(
                        () -> {
                          final var diff =
                              swerve
                                  .getPose()
                                  .minus(currentPath.getInitialPose().orElse(Pose2d.kZero));
                          return MathUtil.isNear(
                                  0.0,
                                  diff.getX(),
                                  Units.inchesToMeters(2.0)) // TODO update when merged
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(2.0))
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
                        })
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> currentPath.getInitialPose().orElse(Pose2d.kZero))),
                currentPath.cmd()));
  }

  // /***
  //  * Runs a path based on the names of the starting location, middle location, and ending
  // location. The current path is generated based on starting + middle and the next from middle +
  // end
  //  * @param routine AutoRoutine these paths are in
  //  * @param startLocation Starting location of the path that just completed
  //  * @param middleLocation Ending location of the path that just completed/starting location of
  // the next one
  //  * @param endLocation Ending location of the next path
  //  * @param steps Relevant AutoTrajectories + names
  //  */
  // public void bindSegment(
  //     AutoRoutine routine,
  //     String startLocation,
  //     String middleLocation,
  //     String endLocation,
  //     HashMap<String, AutoTrajectory> steps,
  //     ReefTarget target) {
  //   cmdThenPath(
  //       routine,
  //       steps.get(startLocation + "to" + middleLocation),
  //       steps.get(middleLocation + "to" + endLocation),
  //       // Commands.waitSeconds(1.0)
  //       startLocation.substring(0, 1).equals("P") // hp station
  //           ? intakeInAuto(steps.get(middleLocation + "to" + endLocation).getFinalPose())
  //               .alongWith(Commands.print("Scoring In Auto"))
  //           : // Commands.waitSeconds(1.0).alongWith(Commands.print(":3"))
  //           scoreInAuto()); // i've been told we're only scoring on l4 in auto
  // }

  // public Command SLMtoICMD() {
  //   final var routine = factory.newRoutine("SLM to I");
  //   HashMap<String, AutoTrajectory> steps =
  //       new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
  //   String[] stops = {
  //     "SLM", "I", "PLO", "L", "PLO", "K", "PLO", "J" // each stop we are going to, in order
  //   }; // i don't love repeating the plos but ???
  //   for (int i = 0; i < stops.length - 1; i++) {
  //     String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
  //     steps.put(
  //         name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
  //   }
  //   routine
  //       .active()
  //       .whileTrue(
  //           Commands.sequence(
  //               steps.get("SLMtoI").resetOdometry(),
  //               steps.get("SLMtoI").cmd())); // runs the very first traj
  //   for (int i = 0; i < stops.length - 2; i++) {
  //     bindSegment(
  //         routine,
  //         stops[i],
  //         stops[i + 1],
  //         stops[i + 2],
  //         steps); // runs each of the following traj + whatever it does at the end of the traj
  //   }
  //   return routine.cmd();
  // }

  public Command LMtoHCMD() {
    final var routine = factory.newRoutine("LM to H");
    final var traj = routine.trajectory("LMtoH");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command RMtoGCmd() {
    final var routine = factory.newRoutine("RM to G");
    final var traj = routine.trajectory("RMtoG");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command LOtoJ() {
    final var routine = factory.newRoutine("LO to J");
    bindElevatorExtension(routine);
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
        .whileTrue(Commands.sequence(steps.get("LOtoJ").resetOdometry(), steps.get("LOtoJ").cmd()));
    routine
        .observe(steps.get("LOtoJ").done())
        .onTrue(scoreInAuto().andThen(steps.get("JtoPLO").cmd()));
    routine
        .observe(steps.get("JtoPLO").done())
        .onTrue(
            intakeInAuto(steps.get("JtoPLO").getFinalPose()).andThen(steps.get("PLOtoK").cmd()));
    routine
        .observe(steps.get("PLOtoK").done())
        .onTrue(scoreInAuto().andThen(steps.get("KtoPLO").cmd()));
    routine
        .observe(steps.get("KtoPLO").done())
        .onTrue(
            intakeInAuto(steps.get("KtoPLO").getFinalPose()).andThen(steps.get("PLOtoL").cmd()));
    routine
        .observe(steps.get("PLOtoL").done())
        .onTrue(scoreInAuto().andThen(steps.get("LtoPLO").cmd()));
    routine
        .observe(steps.get("LtoPLO").done())
        .onTrue(
            intakeInAuto(steps.get("LtoPLO").getFinalPose()).andThen(steps.get("PLOtoA").cmd()));
    routine.observe(steps.get("PLOtoA").done()).onTrue(scoreInAuto());
    return routine.cmd();
  }
  // public Command LOtoJCMD() {
  //   final var routine = factory.newRoutine("LO to J");
  //   HashMap<String, AutoTrajectory> steps =
  //       new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
  //   String[] stops = {
  //     "LO", "J", "PLO", "K", "PLO", "L", "PLO", "A", "PLO" // each stop we are going to, in order
  //   }; // i don't love repeating the plos but ???
  //   for (int i = 0; i < stops.length - 1; i++) {
  //     String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
  //     steps.put(
  //         name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
  //   }
  //   routine
  //       .active()
  //       .whileTrue(
  //           Commands.sequence(
  //               steps.get("LOtoJ").resetOdometry(),
  //               steps.get("LOtoJ").cmd())); // runs the very first traj
  //   for (int i = 0; i < stops.length - 2; i++) {
  //     bindSegment(
  //         routine,
  //         stops[i],
  //         stops[i + 1],
  //         stops[i + 2],
  //         steps); // runs each of the following traj + whatever it does at the end of the traj
  //   }
  //   return routine.cmd();
  // }

  // public Command ROtoECMD() {
  //   final var routine = factory.newRoutine("RO to E");
  //   HashMap<String, AutoTrajectory> steps =
  //       new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
  //   String[] stops = {
  //     "RO", "E", "PRO", "D", "PRO", "C", "PRO", "B", "PRO" // each stop we are going to, in order
  //   }; // i don't love repeating the plos but ???
  //   for (int i = 0; i < stops.length - 1; i++) {
  //     String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
  //     steps.put(
  //         name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
  //   }
  //   routine
  //       .active()
  //       .whileTrue(
  //           Commands.sequence(
  //               steps.get("ROtoE").resetOdometry(),
  //               steps.get("ROtoE").cmd())); // runs the very first traj
  //   for (int i = 0; i < stops.length - 2; i++) {
  //     bindSegment(
  //         routine,
  //         stops[i],
  //         stops[i + 1],
  //         stops[i + 2],
  //         steps,
  //         ReefTarget
  //             .L2); // runs each of the following traj + whatever it does at the end of the traj
  //   }
  //   bindElevatorExtension(routine);
  //   return routine.cmd();
  // }

  // public Command LItoKCMD() {
  //   final var routine = factory.newRoutine("LI to K");
  //   HashMap<String, AutoTrajectory> steps =
  //       new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
  //   String[] stops = {
  //     "LI", "K", "PLI", "L", "PLI", "A", "PLI", "B", "PLI" // each stop we are going to, in order
  //   }; // i don't love repeating the plos but ???
  //   for (int i = 0; i < stops.length - 1; i++) {
  //     String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
  //     steps.put(
  //         name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
  //   }
  //   routine
  //       .active()
  //       .whileTrue(
  //           Commands.sequence(
  //               steps.get("LItoK").resetOdometry(),
  //               steps.get("LItoK").cmd())); // runs the very first traj
  //   for (int i = 0; i < stops.length - 2; i++) {
  //     bindSegment(
  //         routine,
  //         stops[i],
  //         stops[i + 1],
  //         stops[i + 2],
  //         steps); // runs each of the following traj + whatever it does at the end of the traj
  //   }
  //   return routine.cmd();
  // }

  // public Command RItoDCMD() {
  //   final var routine = factory.newRoutine("RI to D");
  //   HashMap<String, AutoTrajectory> steps =
  //       new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
  //   String[] stops = {
  //     "RI", "D", "PRI", "C", "PRI", "B", "PRI", "A", "PRI" // each stop we are going to, in order
  //   }; // i don't love repeating the plos but ???
  //   for (int i = 0; i < stops.length - 1; i++) {
  //     String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
  //     steps.put(
  //         name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
  //   }
  //   routine
  //       .active()
  //       .whileTrue(
  //           Commands.sequence(
  //               steps.get("RItoD").resetOdometry(),
  //               steps.get("RItoD").cmd())); // runs the very first traj
  //   for (int i = 0; i < stops.length - 2; i++) {
  //     bindSegment(
  //         routine,
  //         stops[i],
  //         stops[i + 1],
  //         stops[i + 2],
  //         steps); // runs each of the following traj + whatever it does at the end of the traj
  //   }
  //   return routine.cmd();
  // }

  // public Command scoreInAuto(Supplier<Pose2d> pose, ReefTarget target) {
  //   return AutoAim.translateToPose(swerve, () -> pose.get())
  //       .alongWith(
  //           Commands.waitUntil(
  //                   () -> {
  //                     final var diff = swerve.getPose().minus(pose.get());
  //                     return MathUtil.isNear(
  //                             0.0, diff.getX(), Units.inchesToMeters(2.0)) // TODO find
  // tolerances
  //                         && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(2.0))
  //                         && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0)
  //                         && MathUtil.isNear(
  //                             target.elevatorHeight,
  //                             elevator.getExtensionMeters(),
  //                             Units.inchesToMeters(2));
  //                   })
  //               .andThen(
  //                   Commands.print(":]"),
  //                   Commands.race(
  //                           Commands.waitUntil(() -> !manipulator.getSecondBeambreak()),
  //                           manipulator.setVoltage(target.outtakeSpeed))
  //                       .withTimeout(1)
  //                       .asProxy()))
  //       .finallyDo(
  //           (interrupted) -> System.out.println("Score in auto ends interrupted: " +
  // interrupted));
  // }
  public Command scoreInAuto() {
    return Commands.runOnce(
            () -> {
              autoScore = true;
              Robot.setCurrentTarget(ReefTarget.L4);
            })
        .andThen(
            Commands.waitUntil(() -> !manipulator.getSecondBeambreak())
                .alongWith(
                    Robot.isSimulation()
                        ? Commands.runOnce(() -> manipulator.setSecondBeambreak(false))
                        : Commands.none())
                .andThen(
                    Commands.runOnce(
                        () -> {
                          autoScore = false;
                          autoPreScore = false;
                        }),
                    swerve.driveVelocity(() -> new ChassisSpeeds(-1, 0, 0)).withTimeout(0.25)));
  }

  public Command intakeInAuto(Optional<Pose2d> pose) {
    if (!pose.isPresent()) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Robot.isSimulation()
              ? Commands.runOnce(() -> manipulator.setSecondBeambreak(true))
              : Commands.none(),
          Commands.print("intake - 2nd bb" + manipulator.getSecondBeambreak()),
          AutoAim.translateToPose(swerve, () -> pose.get())
              .until(() -> manipulator.getSecondBeambreak() || manipulator.getFirstBeambreak()));
    }
  }

  public void bindElevatorExtension(AutoRoutine routine) {
    bindElevatorExtension(routine, 3); //TODO tune
  }

  public void bindElevatorExtension(AutoRoutine routine, double toleranceMeters) {
    routine
        .observe(
            () ->
                swerve
                            .getPose()
                            .getTranslation()
                            .minus(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? AutoAim.BLUE_REEF_CENTER
                                    : AutoAim.RED_REEF_CENTER)
                            .getNorm()
                        < toleranceMeters
                    && (manipulator.getSecondBeambreak()))
        .onTrue(Commands.runOnce(() -> autoPreScore = true))
        .onFalse(Commands.runOnce(() -> autoPreScore = false));
  }
}
