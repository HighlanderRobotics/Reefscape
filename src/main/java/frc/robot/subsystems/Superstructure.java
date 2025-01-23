package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeReefTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum SuperState {
    IDLE,
    INTAKE_CORAL_HP,
    INTAKE_CORAL_GROUND,
    READY_CORAL,
    SPIT_CORAL,
    PRE_L1,
    PRE_L2,
    PRE_L3,
    PRE_L4,
    SCORE_CORAL,
    ANTI_JAM,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_HIGH,
    INTAKE_ALGAE_LOW,
    INTAKE_ALGAE_STACK,
    READY_ALGAE,
    SPIT_ALGAE,
    PRE_PROCESSOR,
    PRE_NET,
    SCORE_ALGAE,
    PRE_CLIMB,
    CLIMB
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> chassisVel;
  private final Supplier<ReefTarget> reefTarget;
  private final Supplier<AlgaeReefTarget> algaeReefTarget;

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

  private final ElevatorSubsystem elevator;
  private final ManipulatorSubsystem manipulator;
  // Intake would be included here, but is cut from cad as of rn

  public Superstructure(
      ElevatorSubsystem elevator,
      ManipulatorSubsystem manipulator,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> chassisVel,
      Supplier<ReefTarget> reefTarget,
      Supplier<AlgaeReefTarget> algaeReefTarget,
      Trigger scoreReq,
      Trigger prescoreReq,
      Trigger groundIntakeCoralReq,
      Trigger hpIntakeCoralReq,
      Trigger groundIntakeAlgaeReq,
      Trigger reefIntakeAlgaeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCanReq) {
    this.elevator = elevator;
    this.manipulator = manipulator;

    this.pose = pose;
    this.chassisVel = chassisVel;
    this.reefTarget = reefTarget;
    this.algaeReefTarget = algaeReefTarget;

    this.prescoreReq = prescoreReq;
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

  private void configureStateTransitionCommands() {
    stateTriggers
        .get(SuperState.IDLE)
        // TODO impl other mechanism IDLE states
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.setVelocity(0.0));

    // IDLE to coral intake
    stateTriggers
        .get(SuperState.IDLE)
        .and(hpIntakeCoralReq)
        .onTrue(this.forceState(SuperState.INTAKE_CORAL_HP));

    stateTriggers
        .get(SuperState.IDLE)
        .and(groundIntakeCoralReq)
        .onTrue(this.forceState(SuperState.INTAKE_CORAL_GROUND));

    // IDLE to algae intake
    stateTriggers
        .get(SuperState.IDLE)
        .and(groundIntakeAlgaeReq)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_GROUND));

    stateTriggers
        .get(SuperState.IDLE)
        .and(reefIntakeAlgaeReq)
        .and(() -> algaeReefTarget.get() == AlgaeReefTarget.HIGH)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_HIGH));

    stateTriggers
        .get(SuperState.IDLE)
        .and(reefIntakeAlgaeReq)
        .and(() -> algaeReefTarget.get() == AlgaeReefTarget.LOW)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_LOW));

    stateTriggers
        .get(SuperState.IDLE)
        .and(reefIntakeAlgaeReq)
        .and(() -> algaeReefTarget.get() == AlgaeReefTarget.STACK)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_STACK));
    // IDLE to climb
    stateTriggers
        .get(SuperState.IDLE)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.PRE_CLIMB));
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = nextState;
        });
  }
}
