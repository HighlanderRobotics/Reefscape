package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.ReefTarget;

public class Superstructure {
  public static enum SuperState {
    IDLE
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> chassisVel;

  /** Also triggered by scoreReq */
  private final Trigger prescoreReq;
  private final Trigger scoreReq;

  private final Trigger groundIntakeCoralReq;
  private final Trigger hpIntakeCoralReq;

  private final Trigger groundIntakeAlgaeReq;
  private final Trigger reefIntakeAlgaeReq;

  private final Trigger preClimbReq;
  private final Trigger climbConfReq;
  private final Trigger climbCanReq;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;
  private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

  private Timer stateTimer = new Timer();

  public Superstructure(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> chassisVel,
      Trigger scoreReq,
      Trigger prescoreReq,
      Trigger groundIntakeCoralReq,
      Trigger hpIntakeCoralReq,
      Trigger groundIntakeAlgaeReq,
      Trigger reefIntakeAlgaeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCanReq) {
    this.pose = pose;
    this.chassisVel = chassisVel;

    this.prescoreReq = prescoreReq.or(scoreReq);
    this.scoreReq = scoreReq;

    this.groundIntakeCoralReq = groundIntakeCoralReq;
    this.hpIntakeCoralReq = hpIntakeCoralReq;

    this.groundIntakeAlgaeReq = groundIntakeAlgaeReq;
    this.reefIntakeAlgaeReq = reefIntakeAlgaeReq;

    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCanReq = climbCanReq;

    stateTimer.start();

    for (var state : SuperState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }

    configureStateTransitionCommands();
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    Logger.recordOutput("Superstructure State", state);
  }

  private void configureStateTransitionCommands() {}
}
