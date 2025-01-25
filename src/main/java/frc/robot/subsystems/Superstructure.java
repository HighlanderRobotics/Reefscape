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

  private final Trigger preScoreReq;
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
      Trigger preScoreReq,
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

    this.preScoreReq = preScoreReq;
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

    stateTriggers
        .get(SuperState.INTAKE_CORAL_HP)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.HP_EXTENSION_METERS))
        .whileTrue(manipulator.indexCmd()) // TODO add joints, adjust manipulator behavior
        .and(manipulator::getSecondBeambreak)
        .onTrue(this.forceState(SuperState.READY_CORAL));

    // No-op until intake becomes a thing
    stateTriggers.get(SuperState.INTAKE_CORAL_GROUND).onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.READY_CORAL)
        // TODO add joints, maybe adjust other logic
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.indexCmd()); // keep indexing to make sure its chilling

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preScoreReq)
        .and(() -> reefTarget.get() == ReefTarget.L1)
        .onTrue(this.forceState(SuperState.PRE_L1));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preScoreReq)
        .and(() -> reefTarget.get() == ReefTarget.L2)
        .onTrue(this.forceState(SuperState.PRE_L2));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preScoreReq)
        .and(() -> reefTarget.get() == ReefTarget.L3)
        .onTrue(this.forceState(SuperState.PRE_L3));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preScoreReq)
        .and(() -> reefTarget.get() == ReefTarget.L4)
        .onTrue(this.forceState(SuperState.PRE_L4));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.SPIT_CORAL));

    stateTriggers
        .get(SuperState.PRE_L1)
        // TODO add joints and L1 constant
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(0.0))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L2)
        // TODO add joints
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L3)
        // TODO add joints
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L4)
        // TODO add joints
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));
    
    stateTriggers.get(SuperState.SCORE_CORAL).
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = nextState;
        });
  }
}
