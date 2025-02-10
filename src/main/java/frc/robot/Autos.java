// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
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

  public static boolean autoPreScore = false;
  public static boolean autoScore = false; // TODO perhaps this should not be static
  public static boolean autoAlgaeIntake = false;

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

  public Command LMtoH() {
    final var routine = factory.newRoutine("LM to H");
    final var traj = routine.trajectory("LMtoH");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindElevatorExtension(routine, true);
    routine.observe(traj.done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command RMtoG() {
    final var routine = factory.newRoutine("RM to G");
    final var traj = routine.trajectory("RMtoG");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindElevatorExtension(routine, true);
    routine.observe(traj.done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public void runCoralPath(
      AutoRoutine routine,
      String startPos,
      String endPos,
      String nextPos,
      HashMap<String, AutoTrajectory> steps) {
    routine
        .observe(steps.get(startPos + "to" + endPos).done())
        .onTrue(
            Commands.sequence(
                endPos.length() == 3
                    ? intakeCoralInAuto(steps.get(startPos + "to" + endPos).getFinalPose())
                    : Commands.sequence(
                        endPos.length() == 1 ? scoreCoralInAuto() : Commands.print("pushed bot")),
                steps.get(endPos + "to" + nextPos).cmd()));
  }

  public void runAlgaePath(
      AutoRoutine routine,
      String startPos,
      String endPos,
      String nextPos,
      HashMap<String, AutoTrajectory> steps) {
    routine
        .observe(steps.get(startPos + "to" + endPos).done())
        .onTrue(
            Commands.sequence(
                endPos.equals("CM") // TODO
                    ? scoreAlgaeInAuto()
                    : intakeAlgaeInAuto(
                        steps.get(startPos + "to" + endPos).getFinalPose(),
                        endPos.equals("AB") || endPos.equals("EF") || endPos.equals("IJ")
                            ? AlgaeIntakeTarget.HIGH
                            : AlgaeIntakeTarget
                                .LOW), // just don't worry about it i don't like it either
                steps.get(endPos + "to" + nextPos).cmd()));
  }

  public Command LOtoJ() {
    final var routine = factory.newRoutine("LO to J");
    bindElevatorExtension(routine, true);
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "LO", "J", "PLO", "K", "PLO", "L", "PLO", "A", "PLO" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(name, routine.trajectory(name));
    }
    routine
        // run first path
        .active()
        .whileTrue(Commands.sequence(steps.get("LOtoJ").resetOdometry(), steps.get("LOtoJ").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PLOtoA").done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command ROtoE() {
    final var routine = factory.newRoutine("RO to E");
    bindElevatorExtension(routine, true);
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
        // run first path
        .active()
        .whileTrue(Commands.sequence(steps.get("ROtoE").resetOdometry(), steps.get("ROtoE").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PROtoB").done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command LItoK() {
    final var routine = factory.newRoutine("LI to K");
    bindElevatorExtension(routine, true);
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
        // run first path
        .active()
        .whileTrue(Commands.sequence(steps.get("LItoK").resetOdometry(), steps.get("LItoK").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PLItoB").done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command RItoD() {
    final var routine = factory.newRoutine("RI to D");
    bindElevatorExtension(routine, true);
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
        // run first path
        .active()
        .whileTrue(Commands.sequence(steps.get("RItoD").resetOdometry(), steps.get("RItoD").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PRItoA").done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command PMtoPL() {
    final var routine = factory.newRoutine("PM to PL");
    bindElevatorExtension(routine, true);
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "PM", "PL", "PM", "PR", "I", "PLO", "K", "PLO", "L" // each stop we are going to, in order
    };
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        // run first path
        .active()
        .whileTrue(
            Commands.sequence(steps.get("PMtoPL").resetOdometry(), steps.get("PMtoPL").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PLOtoL").done()).onTrue(scoreCoralInAuto());
    return routine.cmd();
  }

  public Command RStoGH() {
    final var routine = factory.newRoutine("RS to GH");
    bindElevatorExtension(routine, false);
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "RS", "GH", "CM", "IJ", "CM", "EF", "CM" // each stop we are going to, in order
    };
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        // run first path
        .active()
        .whileTrue(
            Commands.sequence(steps.get("RStoGH").resetOdometry(), steps.get("RStoGH").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runAlgaePath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("EFtoCM").done()).onTrue(scoreAlgaeInAuto());
    return routine.cmd();
  }

  public Command scoreCoralInAuto() {
    return Commands.runOnce(
            () -> {
              autoScore = true;
              Robot.setCurrentCoralTarget(ReefTarget.L4);
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

  public Command intakeCoralInAuto(Optional<Pose2d> pose) {
    if (!pose.isPresent()) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Robot.isSimulation()
              ? Commands.runOnce(() -> manipulator.setSecondBeambreak(true))
              : Commands.none(),
          AutoAim.translateToPose(swerve, () -> pose.get())
              .until(() -> manipulator.getSecondBeambreak() || manipulator.getFirstBeambreak()));
    }
  }

  public Command scoreAlgaeInAuto() {
    return Commands.runOnce(
            () -> {
              autoScore = true;
              Robot.setCurrentAlgaeScoreTarget(AlgaeScoreTarget.NET);
            })
        .andThen(
            Commands.waitUntil(() -> !manipulator.hasAlgae())
                .alongWith(
                    Robot.isSimulation()
                        ? Commands.runOnce(() -> manipulator.setHasAlgae(false))
                        : Commands.none())
                .andThen(
                    Commands.runOnce(
                        () -> {
                          autoScore = false;
                          autoPreScore = false;
                        })
                    ));
  }

  public Command intakeAlgaeInAuto(Optional<Pose2d> pose, AlgaeIntakeTarget target) {
    if (!pose.isPresent()) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Commands.runOnce(
              () -> {
                autoAlgaeIntake = true;
                Robot.setCurrentAlgaeIntakeTarget(target);
              }),
          Commands.waitUntil(() -> manipulator.hasAlgae())
              .alongWith(
                  Robot.isSimulation()
                      ? Commands.runOnce(() -> manipulator.setHasAlgae(true))
                      : Commands.none()),
          Commands.runOnce(
              () -> {
                autoAlgaeIntake = false;
              }));
    }
  }

  public void bindElevatorExtension(AutoRoutine routine, boolean coral) {
    bindElevatorExtension(routine, 3, coral); // TODO tune
  }

  public void bindElevatorExtension(AutoRoutine routine, double toleranceMeters, boolean coral) {
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
                    && (coral
                        ? manipulator.getSecondBeambreak()
                        : manipulator.hasAlgae())) // TODO uhhhh
        .onTrue(Commands.runOnce(() -> autoPreScore = true))
        .onFalse(
            Commands.runOnce(
                () ->
                    autoPreScore =
                        false)); // TODO do we need to differentiate between autoprescore coral and
    // autoprescore algae or am i overcooking
  }
}
