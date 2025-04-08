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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.ReefTarget;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.CoralTargets;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final ManipulatorSubsystem manipulator;
  private final FunnelSubsystem funnel;
  private final AutoFactory factory;

  @AutoLogOutput public static boolean autoPreScore = true;
  @AutoLogOutput public static boolean autoScore = false; // TODO perhaps this should not be static
  @AutoLogOutput public static boolean autoGroundIntake = false;

  public Autos(SwerveSubsystem swerve, ManipulatorSubsystem manipulator, FunnelSubsystem funnel) {
    this.swerve = swerve;
    this.manipulator = manipulator;
    this.funnel = funnel;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              if (Robot.ROBOT_TYPE != RobotType.REAL)
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
    bindElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command RMtoG() {
    final var routine = factory.newRoutine("RM to G");
    final var traj = routine.trajectory("RMtoG");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public void runPath(
      AutoRoutine routine,
      String startPos,
      String endPos,
      String nextPos,
      HashMap<String, AutoTrajectory> steps) {
    routine
        .observe(
            steps
                .get(startPos + "to" + endPos)
                .atTime(
                    steps.get(startPos + "to" + endPos).getRawTrajectory().getTotalTime()
                        - (endPos.length() == 1 ? 0.1 : 0.0)))
        .onTrue(
            Commands.sequence(
                endPos.length() == 3
                    ? intakeInAuto(() -> steps.get(startPos + "to" + endPos).getFinalPose())
                    : Commands.sequence(
                        endPos.length() == 1
                            ? scoreInAuto(
                                () -> steps.get(startPos + "to" + endPos).getFinalPose().get())
                            : AutoAim.translateToPose(
                                    swerve,
                                    () -> steps.get(startPos + "to" + endPos).getFinalPose().get())
                                .until(
                                    () ->
                                        AutoAim.isInTolerance(
                                            swerve.getPose(),
                                            steps
                                                .get(startPos + "to" + endPos)
                                                .getFinalPose()
                                                .get()))
                                .withTimeout(2.0)),
                steps.get(endPos + "to" + nextPos).cmd()));
  }

  public void runGroundPath(AutoRoutine routine,
  String startPos,
  String endPos,
  String nextPos,
  HashMap<String, AutoTrajectory> steps) {
    routine
        .observe(
            steps
                .get(startPos + "to" + endPos).done())
                .onTrue(Commands.sequence(
                  scoreInAuto(() -> steps.get(startPos + "to" + endPos).getFinalPose().get()),
                  Commands.runOnce(() -> autoGroundIntake = true),
                  steps.get(endPos + "to" + nextPos).cmd()));
  }

  public Command LOtoJ() {
    final var routine = factory.newRoutine("LO to J");
    bindElevatorExtension(routine);
    routine.active().onTrue(Commands.print("Auto!"));
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "LO", "J", "PLO", "K", "PLO", "L", "PLM", "A", "PLO" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(name, routine.trajectory(name));
    }
    routine
        // run first path
        .active()
        .onTrue(Commands.runOnce(() -> Robot.setCurrentTarget(ReefTarget.L4)))
        .whileTrue(Commands.sequence(steps.get("LOtoJ").resetOdometry(), steps.get("LOtoJ").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runPath(routine, startPos, endPos, nextPos, steps);
    }

    // final var groundTraj = routine.trajectory("LtoAGround");

    // routine
    //     .observe(steps.get("PLOtoL").recentlyDone())
    //     .onTrue(scoreInAuto(() -> steps.get("PLOtoL").getFinalPose().get()))
    //     .and(() -> !manipulator.getSecondBeambreak() && !manipulator.getFirstBeambreak())
    //     .onTrue(Commands.runOnce(() -> Robot.setCurrentTarget(ReefTarget.L2)))
    //     .onTrue(groundTraj.cmd().andThen(scoreInAuto(() -> groundTraj.getFinalPose().get())))
    //     .onTrue(Commands.runOnce(() -> autoGroundIntake = true));

    // routine
    //     .observe(groundTraj.done())
    //     .onTrue(Commands.runOnce(() -> autoGroundIntake = false).ignoringDisable(true));

    routine
        .observe(steps.get("LtoPLM").done())
        .onTrue(Commands.runOnce(() -> Robot.setCurrentTarget(ReefTarget.L2)));

    return routine.cmd().alongWith(Commands.print("auto :("));
  }

  public Command ROtoE() {
    final var routine = factory.newRoutine("RO to E");
    bindElevatorExtension(routine);
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "RO", "E", "PRO", "D", "PRO", "C", "PRM", "B", "PRO" // each stop we are going to, in order
    }; // i don't love repeating the plos but ???
    for (int i = 0; i < stops.length - 1; i++) {
      String name = stops[i] + "to" + stops[i + 1]; // concatenate the names of the stops
      steps.put(
          name, routine.trajectory(name)); // and puts that name + corresponding traj to the map
    }
    routine
        // run first path
        .active()
        .onTrue(Commands.runOnce(() -> Robot.setCurrentTarget(ReefTarget.L4)))
        .whileTrue(Commands.sequence(steps.get("ROtoE").resetOdometry(), steps.get("ROtoE").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runPath(routine, startPos, endPos, nextPos, steps);
    }

    routine
        .observe(steps.get("CtoPRM").done())
        .onTrue(Commands.runOnce(() -> Robot.setCurrentTarget(ReefTarget.L2)));

    return routine.cmd();
  }

  public Command LItoK() {
    final var routine = factory.newRoutine("LI to K");
    bindElevatorExtension(routine);
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
      runPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PLItoB").done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command RItoD() {
    final var routine = factory.newRoutine("RI to D");
    bindElevatorExtension(routine);
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
      runPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PRItoA").done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command PMtoPL() {
    final var routine = factory.newRoutine("PM to PL");
    bindElevatorExtension(routine, 2.0);
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
      runPath(routine, startPos, endPos, nextPos, steps);
    }
    // final path
    routine.observe(steps.get("PLOtoL").done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command LOtoA() { //2910
    final var routine = factory.newRoutine("LO to A");
    bindElevatorExtension(routine, 2.0);
    HashMap<String, AutoTrajectory> steps =
    new HashMap<String, AutoTrajectory>();
    // lo a b4 b2 dealgae
    routine
        // run first path
        .active()
        .whileTrue(
            Commands.sequence(routine.trajectory("LOtoA").resetOdometry(), routine.trajectory("LOtoA").cmd()));
    runGroundPath(routine, "LO", "A", "B", steps);
    
    routine.observe(routine.trajectory("LOtoA").done()).onTrue(scoreInAuto());
    return routine.cmd();
  }

  public Command scoreInAuto(Supplier<Pose2d> trajEndPose) {
    return Commands.sequence(
            Commands.waitUntil(
                new Trigger(
                        () ->
                            AutoAim.isInTolerance(
                                    swerve.getPose(),
                                    CoralTargets.getClosestTarget(trajEndPose.get()),
                                    swerve.getVelocityFieldRelative(),
                                    Units.inchesToMeters(1.0),
                                    Units.degreesToRadians(1.0))
                                && MathUtil.isNear(
                                    0,
                                    Math.hypot(
                                        swerve.getVelocityRobotRelative().vxMetersPerSecond,
                                        swerve.getVelocityRobotRelative().vyMetersPerSecond),
                                    AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND)
                                && MathUtil.isNear(
                                    0.0,
                                    swerve.getVelocityRobotRelative().omegaRadiansPerSecond,
                                    3.0))
                    .debounce(0.10)),
            Commands.print("Scoring!"),
            Commands.runOnce(
                () -> {
                  autoScore = true;
                  // Robot.setCurrentTarget(ReefTarget.L4);
                }),
            Commands.waitUntil(() -> !manipulator.getSecondBeambreak())
                .alongWith(
                    Robot.isSimulation()
                        ? Commands.runOnce(() -> manipulator.setSecondBeambreak(false))
                        : Commands.none()),
            Commands.runOnce(
                () -> {
                  autoScore = false;
                  autoPreScore = false;
                }))
        .raceWith(
            AutoAim.translateToPose(
                swerve,
                () -> CoralTargets.getClosestTarget(trajEndPose.get()),
                ChassisSpeeds::new,
                new Constraints(1.5, 2.0)));
  }

  // TODO: REMOVE THIS OVERLOAD
  public Command scoreInAuto() {
    return scoreInAuto(() -> swerve.getPose());
  }

  public Command intakeInAuto(Supplier<Optional<Pose2d>> pose) {
    if (!pose.get().isPresent()) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Robot.isSimulation()
              ? Commands.runOnce(() -> manipulator.setSecondBeambreak(true))
              : Commands.none(),
          Commands.print("intake - 2nd bb" + manipulator.getSecondBeambreak()),
          // AutoAim.translateToPose(
          //         swerve,
          //         () -> pose.get().get(),
          //         () ->
          //             ChassisSpeeds.fromRobotRelativeSpeeds(
          //                 new ChassisSpeeds(-0.5, 0.0, 0.0), swerve.getRotation()))
          swerve
              .driveVoltage(() -> new ChassisSpeeds(-0.0, 0.0, 0.0))
              .until(
                  () ->
                      manipulator.getSecondBeambreak()
                          || manipulator.getFirstBeambreak()
                          || funnel.getFilteredCurrent() > 20.0));
    }
  }

  public void bindElevatorExtension(AutoRoutine routine) {
    bindElevatorExtension(routine, 3.5); // TODO tune
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
        .whileTrue(Commands.run(() -> autoPreScore = true))
        .whileFalse(Commands.run(() -> autoPreScore = false));
  }
}
