// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.LinkedList;

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
            //getcoral
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
                PROtoD.cmd())); //should be DtoPRO instead?

    return routine.cmd();
  }


  public Command SLMtoICMD() {
    final var routine = factory.newRoutine("SLM to I");

    LinkedList<AutoTrajectory> steps = new LinkedList<AutoTrajectory>();
    String[] stepNames = {"SLMtoI","ItoPLO", "PLOtoL", "LtoPLO", "PLOtoK", "KtoPLO"};
    for(String name:stepNames) {
      steps.add(routine.trajectory(name));
    }

    routine
    .observe(steps.get(0).done()) //SLMtoI
    .onTrue(
        Commands.sequence(
            // score
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(1).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(1).cmd())); //ItoPLO
routine
    .observe(steps.get(1).done())
    .onTrue(
        Commands.sequence(
            // intake
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(2).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(2).cmd())); //PLotoL
routine
    .observe(steps.get(2).done())
    .onTrue(
        Commands.sequence(
            // score
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(3).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(3).cmd())); //LtoPLO
routine
    .observe(steps.get(3).done())
    .onTrue(
        Commands.sequence(
            // intake
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(2).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(2).cmd())); //PLotoL
routine
    .observe(steps.get(2).done())
    .onTrue(
        Commands.sequence(
            // score
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(3).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(3).cmd())); //LtoPLO
routine
    .observe(steps.get(3).done())
    .onTrue(
        Commands.sequence(
            // intake
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(4).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(4).cmd())); //PLOtoK
routine
    .observe(steps.get(4).done())
    .onTrue(
        Commands.sequence(
            // score
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(5).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(5).cmd())); //KtoPLO
routine
    .observe(steps.get(3).done())
    .onTrue(
        Commands.sequence(
            // intake
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(4).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(4).cmd())); //PLOtoK
routine
    .observe(steps.get(4).done())
    .onTrue(
        Commands.sequence(
            // score
            Commands.waitSeconds(0.5)
                .raceWith(
                    swerve.poseLockDriveCommand(
                        () -> steps.get(5).getInitialPose().orElse(Pose2d.kZero))),
            steps.get(5).cmd())); //KtoPLO

    return routine.cmd();
   }

  /* public Command CircleReef() {
    final var routine = factory.newRoutine("Circle the reef, scoring on L4");

    LinkedList<AutoTrajectory> steps = new LinkedList<AutoTrajectory>();
    String[] stepNames = {"RStoG","GtoPRO", "PROtoF", "FtoPRO", "PROtoE",
    "EtoPRO", "PROtoD", "DtoPRO", "PROtoC", "CtoPRI", "PRItoB", "BtoPRI",
    "PRItoA", "AtoPLI", "PLItoL", "LtoPLO", "PLOtoK", "KtoPLO", "PLOtoJ", 
    "JtoPLO", "PLOtoI", "ItoPLO", "PLOtpH"};
    for(String name:stepNames) {
      steps.add(routine.trajectory(name));
    }
    
   AutoTrajectory previous=null;
   for(AutoTrajectory current:steps) {
    if(previous==null) {
      routine.active().whileTrue(Commands.sequence(current.resetOdometry(), current.cmd()));
    } else {
      routine
      .observe(previous.done())
      .onTrue(
          Commands.sequence(
              // score
              Commands.waitSeconds(0.5)
                  .raceWith(
                      swerve.poseLockDriveCommand(
                          () -> current.getInitialPose().orElse(Pose2d.kZero))),
                          current.cmd()));       
    }
    previous=current;
   }
    */


    // Steps with numeric part in name
    /* 
    for(int i=0;i<30;i++) {
      list.add(routine.trajectory("Step_"+i));
    }
    */   
/* 

    routine.active().whileTrue(Commands.sequence(list.get(0).resetOdometry(), list.get(0).cmd()));

    //for(var obj:list) {
      //obj would be current item
    //}

    // Loop only works if list has at least 2 items
    for(int i=0;i<list.size()-1;i++) {
      final var current=list.get(i);
      final var next=list.get(i+1);

      routine
      .observe(current.done())
      .onTrue(
          Commands.sequence(
              // score
              Commands.waitSeconds(0.5)
                  .raceWith(
                      swerve.poseLockDriveCommand(
                          () -> next.getInitialPose().orElse(Pose2d.kZero))),
                          next.cmd()));      
    }

*/
  //  return routine.cmd();
 // }

}
