// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem.ShoulderState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.autoaim.AutoAim;
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
  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;

  @AutoLogOutput public static boolean autoPreScore = true;
  @AutoLogOutput public static boolean autoScore = false; // TODO perhaps this should not be static
  @AutoLogOutput public static boolean autoGroundCoralIntake = false;
  @AutoLogOutput public static boolean autoAlgaeIntake = false;

  public Autos(
      SwerveSubsystem swerve,
      ManipulatorSubsystem manipulator,
      FunnelSubsystem funnel,
      ElevatorSubsystem elevator,
      ShoulderSubsystem shoulder,
      WristSubsystem wrist) {
    this.swerve = swerve;
    this.manipulator = manipulator;
    this.funnel = funnel;
    this.elevator = elevator;
    this.shoulder = shoulder;
    this.wrist = wrist;
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
    bindCoralElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreCoralInAuto(swerve::getPose));
    return routine.cmd();
  }

  public Command RMtoG() {
    final var routine = factory.newRoutine("RM to G");
    final var traj = routine.trajectory("RMtoG");
    routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    bindCoralElevatorExtension(routine);
    routine.observe(traj.done()).onTrue(scoreCoralInAuto(swerve::getPose));
    return routine.cmd();
  }

  public void runCoralPath(
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
                        - (endPos.length() == 1 ? 0.3 : 0.0)))
        .onTrue(
            Commands.sequence(
                endPos.length() == 3
                    ? intakeCoralInAuto(() -> steps.get(startPos + "to" + endPos).getFinalPose())
                    : scoreCoralInAuto(
                        () -> steps.get(startPos + "to" + endPos).getFinalPose().get()),
                steps.get(endPos + "to" + nextPos).cmd()));
  }

  public Command LOtoJ() {
    final var routine = factory.newRoutine("LO to J");
    bindCoralElevatorExtension(routine);
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
        .onTrue(Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L4)))
        .whileTrue(Commands.sequence(steps.get("LOtoJ").resetOdometry(), steps.get("LOtoJ").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
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
        .onTrue(Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L2)));

    return routine.cmd().alongWith(Commands.print("auto :("));
  }

  public Command ROtoE() {
    final var routine = factory.newRoutine("RO to E");
    bindCoralElevatorExtension(routine);
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
        .onTrue(Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L4)))
        .whileTrue(Commands.sequence(steps.get("ROtoE").resetOdometry(), steps.get("ROtoE").cmd()));
    // run middle paths
    // and puts that name + corresponding traj to the map
    for (int i = 0; i < stops.length - 2; i++) {
      String startPos = stops[i];
      String endPos = stops[i + 1];
      String nextPos = stops[i + 2];
      runCoralPath(routine, startPos, endPos, nextPos, steps);
    }

    routine
        .observe(steps.get("CtoPRM").done())
        .onTrue(Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L2)));

    return routine.cmd();
  }

  public Command LItoK() {
    final var routine = factory.newRoutine("LI to K");
    bindCoralElevatorExtension(routine);
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
    routine.observe(steps.get("PLItoB").done()).onTrue(scoreCoralInAuto(swerve::getPose));
    return routine.cmd();
  }

  public Command RItoD() {
    final var routine = factory.newRoutine("RI to D");
    bindCoralElevatorExtension(routine);
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
    routine.observe(steps.get("PRItoA").done()).onTrue(scoreCoralInAuto(swerve::getPose));
    return routine.cmd();
  }

  public Command PMtoPL() {
    final var routine = factory.newRoutine("PM to PL");
    bindCoralElevatorExtension(routine, 2.0);
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
    routine.observe(steps.get("PLOtoL").done()).onTrue(scoreCoralInAuto(swerve::getPose));
    return routine.cmd();
  }

  public Command CMtoGH() { // algae path
    final var routine = factory.newRoutine("CM to GH");
    bindCoralElevatorExtension(routine, 2.0);
    bindAlgaeElevatorExtension(routine);
    HashMap<String, AutoTrajectory> steps =
        new HashMap<String, AutoTrajectory>(); // key - name of path, value - traj
    String[] stops = {
      "CM", "G", "GH", "NI", "IJ", "NI", "EF" // each stop we are going to, in order
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
            Commands.sequence(
                steps.get("CMtoG").resetOdometry(),
                Commands.waitSeconds(1.5),
                steps.get("CMtoG").cmd()));

    routine
        .observe(steps.get("CMtoG").done()) // TODO change to time based
        .onTrue(Commands.sequence(scoreCoralInAuto(() -> steps.get("CMtoG").getFinalPose().get())));
    routine
        .observe(() -> !manipulator.getFirstBeambreak() && !manipulator.getSecondBeambreak())
        .onTrue(
            Commands.sequence(
                swerve.driveTeleop(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(0.2),
                intakeAlgaeInAuto(() -> steps.get("CMtoG").getFinalPose()),
                steps.get("GHtoNI").cmd()));
    routine
        .observe(
            steps
                .get("GHtoNI")
                .atTime(steps.get("GHtoNI").getRawTrajectory().getTotalTime() - 0.2)) // TODO tune
        .onTrue(Commands.sequence(scoreAlgaeInAuto(), steps.get("NItoIJ").cmd()));
    // ------------------sketchy--------
    routine
        .observe(steps.get("NItoIJ").done())
        .onTrue(
            Commands.sequence(
                intakeAlgaeInAuto(() -> steps.get("NItoIJ").getFinalPose()),
                swerve.driveTeleop(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(0.2)));

    // routine
    //     .observe(
    //         steps
    //             .get("IJtoNI")
    //             .atTime(steps.get("IJtoNI").getRawTrajectory().getTotalTime() - 0.2)) // TODO
    // tune
    //     .onTrue(Commands.sequence(scoreAlgaeInAuto(), steps.get("NItoEF").cmd()));
    // routine
    //     .observe(steps.get("NItoEF").done())
    //     .onTrue(Commands.sequence(intakeAlgaeInAuto(() -> steps.get("NItoEF").getFinalPose())));

    // ---------------------------
    // Commands.sequence(
    //     // AutoAim.translateToXCoord(
    //     //         swerve,
    //     //         () ->
    //     //             DriverStation.getAlliance().get() == Alliance.Blue
    //     //                 ? AutoAim.BLUE_NET_X
    //     //                 : AutoAim.RED_NET_X,
    //     //         () -> 0,
    //     //         () ->
    //     //             DriverStation.getAlliance().get() == Alliance.Blue
    //     //                 ? Rotation2d.k180deg
    //     //                 : Rotation2d.kZero.plus(Rotation2d.fromDegrees(20.0)))
    //         AutoAim.translateToPose(swerve,
    //         () -> new Pose2d(DriverStation.getAlliance().get() == Alliance.Blue
    //                         ? AutoAim.BLUE_NET_X
    //                         : AutoAim.RED_NET_X,
    //                         steps.get("GhtoNI").))
    //         .until(
    //             () ->
    //                 (MathUtil.isNear(
    //                         DriverStation.getAlliance().get() == Alliance.Blue
    //                             ? AutoAim.BLUE_NET_X
    //                             : AutoAim.RED_NET_X,
    //                         swerve.getPose().getX(),
    //                         Units.inchesToMeters(5))
    //                     && MathUtil.isNear(
    //                         DriverStation.getAlliance().get() == Alliance.Blue
    //                             ? Rotation2d.k180deg
    //                                 .plus(Rotation2d.fromDegrees(20.0))
    //                                 .getDegrees()
    //                             : Rotation2d.kZero
    //                                 .plus(Rotation2d.fromDegrees(20.0))
    //                                 .getDegrees(),
    //                         swerve.getPose().getRotation().getDegrees(),
    //                         5.0))),
    // scoreAlgaeInAuto())
    // )

    // routine.observe(steps.get("NItoIJ").done()).; //TODO cancel into autoalign

    // for (int i = 0; i < stops.length - 2; i++) {
    //   String startPos = stops[i];
    //   String endPos = stops[i + 1];
    //   String nextPos = stops[i + 2];
    //   runAlgaePath(routine, startPos, endPos, nextPos, steps);
    // }
    // routine
    //     .observe(steps.get("NItoEF").done())
    //     .onTrue(intakeAlgaeInAuto(() -> steps.get("NItoEF").getFinalPose()));
    return routine.cmd();
  }

  public Command LOtoA() { // 2910
    final var routine = factory.newRoutine("LO to A");
    bindCoralElevatorExtension(routine, 2.0);
    HashMap<String, AutoTrajectory> steps = new HashMap<String, AutoTrajectory>();
    // lo a b4 b2 dealgae
    steps.put("LOtoA", routine.trajectory("LOtoA"));
    steps.put("AtoB", routine.trajectory("AtoB"));
    steps.put("BtoB", routine.trajectory("BtoB"));

    // if (Robot.isSimulation()) manipulator.setSimSecondBeambreak(true); // gah //TODO why did i do
    // this????
    routine
        // run first path
        .active()
        .onTrue(Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L4)))
        .whileTrue(Commands.sequence(steps.get("LOtoA").resetOdometry(), steps.get("LOtoA").cmd()));
    routine
        .observe(
            steps.get("LOtoA").atTime(steps.get("LOtoA").getRawTrajectory().getTotalTime() - 0.3))
        .onTrue(
            Commands.sequence(
                scoreCoralInAuto(() -> steps.get("LOtoA").getFinalPose().get()),
                AutoAim.translateToPose(swerve, () -> steps.get("AtoB").getInitialPose().get())
                    .until(
                        () ->
                            elevator.atExtension(ElevatorState.HP.getExtensionMeters())
                                && AutoAim.isInTolerance(
                                    swerve.getPose(), steps.get("AtoB").getInitialPose().get())),
                Commands.runOnce(() -> Robot.setCoralTarget(ReefTarget.L2)),
                steps.get("AtoB").cmd()));
    routine
        .observe(
            steps
                .get("AtoB")
                .active()
                .and(() -> manipulator.getSecondBeambreak() || manipulator.getFirstBeambreak()))
        .onTrue(
            Commands.runOnce(
                () -> {
                  autoGroundCoralIntake = false;
                  Robot.setCoralTarget(ReefTarget.L4);
                }));

    routine
        .observe(
            steps.get("AtoB").atTime(steps.get("AtoB").getRawTrajectory().getTotalTime() - 0.3))
        .onTrue(
            Commands.sequence(
                scoreCoralInAuto(() -> steps.get("AtoB").getFinalPose().get()),
                AutoAim.translateToPose(swerve, () -> steps.get("BtoB").getInitialPose().get())
                    .until(
                        () ->
                            elevator.atExtension(ElevatorState.HP.getExtensionMeters())
                                && AutoAim.isInTolerance(
                                    swerve.getPose(), steps.get("BtoB").getInitialPose().get())),
                Commands.runOnce(() -> autoGroundCoralIntake = true),
                steps.get("BtoB").cmd()));

    // routine
    //     .observe(() -> !manipulator.getFirstBeambreak() && !manipulator.getSecondBeambreak())
    //     .onTrue(
    //         Commands.sequence(
    //             swerve.driveTeleop(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(0.2)));
    // runGroundPath(routine, "LO", "A", "B", steps);
    // runCoralPath(routine, "LO", "A", "B", steps);
    // ----------------
    // runGroundPath(routine, "A", "B", "B", steps);
    // TODO dealgae - merge from prechamps

    // ---------
    return routine.cmd();
  }

  public void runGroundPath(
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
                        - (endPos.length() == 1 ? 0.3 : 0.0)))
        .onTrue(
            Commands.sequence(
                scoreCoralInAuto(() -> steps.get(startPos + "to" + endPos).getFinalPose().get())));
  }

  public Command scoreCoralInAuto(Supplier<Pose2d> trajEndPose) {
    return Commands.sequence(
            Commands.print("scoring"),
            Commands.waitUntil(
                new Trigger(
                        () ->
                            AutoAim.isInTolerance(
                                    swerve.getPose(),
                                    FieldUtils.CoralTargets.getClosestTarget(trajEndPose.get()),
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
                    .debounce(0.06 * 2)),
            Commands.print("Scoring!"),
            Commands.runOnce(
                () -> {
                  autoScore = true;
                  // Robot.setCurrentTarget(ReefTarget.L4);
                }),
            Commands.waitUntil(() -> !manipulator.getSecondBeambreak())
                .alongWith(
                    Robot.isSimulation()
                        ? Commands.runOnce(() -> manipulator.setSimSecondBeambreak(false))
                        : Commands.none()),
            Commands.runOnce(
                () -> {
                  autoScore = false;
                  autoPreScore = false;
                }))
        .raceWith(
            Commands.print("autoaiming frrr")
                .andThen(
                    AutoAim.translateToPose(
                        swerve,
                        () -> FieldUtils.CoralTargets.getClosestTarget(trajEndPose.get()),
                        ChassisSpeeds::new,
                        new Constraints(1.5, 1.0))));
  }

  public Command intakeCoralInAuto(Supplier<Optional<Pose2d>> pose) {
    if (!pose.get().isPresent()) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Robot.isSimulation()
              ? Commands.runOnce(() -> manipulator.setSimSecondBeambreak(true))
              : Commands.none(),
          Commands.print("intake - 2nd bb" + manipulator.getSecondBeambreak()),
          AutoAim.translateToPose(
                  swerve,
                  () -> pose.get().get(),
                  () ->
                      ChassisSpeeds.fromRobotRelativeSpeeds(
                          new ChassisSpeeds(-0.5, 0.0, 0.0), swerve.getRotation()))
              // swerve
              //     .driveVoltage(() -> new ChassisSpeeds(-0.0, 0.0, 0.0))
              .until(
                  () ->
                      manipulator.getSecondBeambreak()
                          || manipulator.getFirstBeambreak()
                          || funnel.getFilteredCurrent() > 20.0));
    }
  }

  public void bindCoralElevatorExtension(AutoRoutine routine) {
    bindCoralElevatorExtension(routine, 3.75); // TODO tune
  }

  public void bindCoralElevatorExtension(AutoRoutine routine, double toleranceMeters) {
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

  public void bindGroundCoralElevatorExtension(
      AutoRoutine routine, double toleranceMeters, Trigger trigger) {
    routine.observe(trigger).debounce(3).onTrue(Commands.run(() -> autoGroundCoralIntake = true));
  }

  // ([[[){ ([[[){ ([[[){ ([[[){ ([[[){ ([[[){ ([[[){ ([[[){ ([[[){ ([[[){

  public void bindAlgaeElevatorExtension(AutoRoutine routine, double toleranceMeters) {
    routine
        .observe(
            () ->
                MathUtil.isNear(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? AutoAim.BLUE_NET_X
                            : AutoAim.RED_NET_X,
                        swerve.getPose().getX(),
                        toleranceMeters)
                    && (manipulator.hasAlgae()))
        .whileTrue(Commands.run(() -> autoPreScore = true))
        .whileFalse(Commands.run(() -> autoPreScore = false));
  }

  public void bindAlgaeElevatorExtension(AutoRoutine routine) {
    bindAlgaeElevatorExtension(routine, 0.2); // TODO tune - is the coral one still requiring atp??
  }

  public Command scoreAlgaeInAuto() { // oh good lord
    return Commands.sequence(
            Commands.waitUntil(
                new Trigger(
                    () ->
                        MathUtil.isNear(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? AutoAim.BLUE_NET_X
                                    : AutoAim.RED_NET_X,
                                swerve.getPose().getX(),
                                Units.inchesToMeters(3.0))
                            && MathUtil.isNear(
                                0,
                                Math.hypot(
                                    swerve.getVelocityRobotRelative().vxMetersPerSecond,
                                    swerve.getVelocityRobotRelative().vyMetersPerSecond),
                                AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND)
                            && MathUtil.isNear(
                                0.0, swerve.getVelocityRobotRelative().omegaRadiansPerSecond, 3.0))
                // .debounce(0.06)), // TODO
                ),
            Commands.print("Scoring algae"),
            Commands.runOnce(
                () -> {
                  Robot.setAlgaeScoreTarget(AlgaeScoreTarget.BARGE);
                  autoScore = true;
                }),
            Commands.waitUntil(() -> !manipulator.hasAlgae())
                .alongWith(
                    Robot.isSimulation()
                        ? Commands.runOnce(() -> manipulator.setSimHasAlgae(false))
                        : Commands.none())
                .andThen(
                    Commands.runOnce(
                        () -> {
                          autoScore = false;
                          autoPreScore = false;
                        })))
        .raceWith(
            AutoAim.translateToXCoord(
                swerve,
                () ->
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? AutoAim.BLUE_NET_X
                        : AutoAim.RED_NET_X,
                () -> 0,
                () ->
                    (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? Rotation2d.k180deg
                            : Rotation2d.kZero)
                        .plus(Rotation2d.fromDegrees(30.0))));
  }

  public Command intakeAlgaeInAuto(Supplier<Optional<Pose2d>> pose) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  autoAlgaeIntake = true;
                  Robot.setAlgaeIntakeTarget(
                      FieldUtils.AlgaeIntakeTargets.getClosestTarget(pose.get().get()) // wow
                          .height);
                }),
            AutoAim.translateToPose(
                    swerve,
                    () ->
                        FieldUtils.AlgaeIntakeTargets.getOffsetLocation(
                            FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(pose.get().get())))
                .until(
                    () ->
                        AutoAim.isInTolerance(
                                swerve.getPose(),
                                FieldUtils.AlgaeIntakeTargets.getOffsetLocation(
                                    FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(
                                        swerve.getPose())),
                                swerve.getVelocityFieldRelative(),
                                Units.inchesToMeters(0.5),
                                Units.degreesToRadians(1.0))
                            && elevator.atExtension()
                            && shoulder.isNearAngle(ShoulderState.INTAKE_ALGAE_REEF.getAngle())
                            && wrist.isNearAngle(WristState.INTAKE_ALGAE_REEF.getAngle())),
            AutoAim.approachAlgae(
                    swerve,
                    () -> FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose()),
                    1)
                .withTimeout(0.5))
        .andThen(
            Commands.runOnce(
                () -> {
                  autoAlgaeIntake = false;
                }));
  }
}
