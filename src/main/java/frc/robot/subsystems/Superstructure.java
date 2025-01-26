package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
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
    PRE_PROCESSOR, // petro
    PRE_NET, // petro
    SCORE_ALGAE, // sam
    PRE_CLIMB, // sam
    CLIMB // sam
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> chassisVel;
  private final Supplier<ReefTarget> reefTarget;
  private final Supplier<AlgaeTarget> algaeTarget;

  private final Trigger preScoreReq;
  private final Trigger scoreReq;

  private final Trigger groundIntakeCoralReq;
  private final Trigger hpIntakeCoralReq;

  private final Trigger intakeAlgaeReq;

  private final Trigger preClimbReq;
  private final Trigger climbConfReq;
  private final Trigger climbCanReq;

  private final Trigger antiJamReq;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;
  private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ManipulatorSubsystem manipulator;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  // Intake would be included here, but is cut from cad as of rn

  public Superstructure(
      ElevatorSubsystem elevator,
      ManipulatorSubsystem manipulator,
      ShoulderSubsystem shoulder,
      WristSubsystem wrist,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> chassisVel,
      Supplier<ReefTarget> reefTarget,
      Supplier<AlgaeTarget> algaeTarget,
      Trigger scoreReq,
      Trigger preScoreReq,
      Trigger groundIntakeCoralReq,
      Trigger hpIntakeCoralReq,
      Trigger intakeAlgaeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCanReq,
      Trigger antiJamReq) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.shoulder = shoulder;
    this.wrist = wrist;

    this.pose = pose;
    this.chassisVel = chassisVel;
    this.reefTarget = reefTarget;
    this.algaeTarget = algaeTarget;

    this.preScoreReq = preScoreReq;
    this.scoreReq = scoreReq;

    this.groundIntakeCoralReq = groundIntakeCoralReq;
    this.hpIntakeCoralReq = hpIntakeCoralReq;

    this.intakeAlgaeReq = intakeAlgaeReq;

    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCanReq = climbCanReq;

    this.antiJamReq = antiJamReq;

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
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.setVelocity(0.0))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS));

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
        .and(intakeAlgaeReq)
        .and(() -> algaeTarget.get() == AlgaeTarget.GROUND)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_GROUND));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeTarget.get() == AlgaeTarget.HIGH)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_HIGH));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeTarget.get() == AlgaeTarget.LOW)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_LOW));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeTarget.get() == AlgaeTarget.STACK)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_STACK));
    // IDLE to climb
    stateTriggers
        .get(SuperState.IDLE)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.PRE_CLIMB));

    // IDLE to ANTI_JAM
    stateTriggers.get(SuperState.IDLE).and(antiJamReq).onTrue(forceState(SuperState.ANTI_JAM));

    stateTriggers
        .get(SuperState.INTAKE_CORAL_HP)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.HP_EXTENSION_METERS))
        .whileTrue(manipulator.index()) // TODO add joints, adjust manipulator behavior
        .and(manipulator::getSecondBeambreak)
        .onTrue(this.forceState(SuperState.READY_CORAL));

    // No-op until intake becomes a thing
    stateTriggers.get(SuperState.INTAKE_CORAL_GROUND).onTrue(this.forceState(SuperState.IDLE));
    // Intake coral ground to anti jam
    stateTriggers
        .get(SuperState.INTAKE_CORAL_GROUND)
        .and(antiJamReq)
        .onTrue(forceState(SuperState.ANTI_JAM));

    stateTriggers
        .get(SuperState.READY_CORAL)
        // TODO add joints, maybe adjust other logic
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.index()); // keep indexing to make sure its chilling
    // Ready coral to anti jam
    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(antiJamReq)
        .onTrue(forceState(SuperState.ANTI_JAM));

    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .whileTrue(manipulator.setVelocity(10))
        .and(manipulator::getFirstBeambreak)
        .negate()
        .and(manipulator::getSecondBeambreak)
        .negate()
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .and(manipulator::getFirstBeambreak)
        .negate()
        .and(manipulator::getSecondBeambreak)
        .negate()
        .and(preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.setVelocity(0.0))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS));

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

    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .whileTrue(manipulator.setVelocity(() -> elevator.isNearExtension(0.0) ? 12.0 : 100.0))
        .and(() -> !manipulator.getSecondBeambreak())
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.ANTI_JAM)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .onFalse(forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(shoulder.setTargetAngle(Rotation2d.fromDegrees(180 - 26.2)))
        .whileTrue(wrist.setTargetAngle(Rotation2d.fromDegrees(360 - 164.9)))
        .whileTrue(manipulator.index())
        .and(() -> manipulator.getFirstBeambreak())
        .onTrue(this.forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_LOW)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .whileTrue(manipulator.index())
        .and(() -> manipulator.getFirstBeambreak())
        .onTrue(this.forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_HIGH)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .whileTrue(manipulator.index())
        // TODO: ADD MANIPULATOR ALGAE BEAMBREAK
        .and(manipulator::getFirstBeambreak)
        .onTrue(forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_STACK)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .whileTrue(manipulator.index())
        // TODO: ADD MANIPULATOR ALGAE BEAMBREAK
        .and(manipulator::getFirstBeambreak)
        .onTrue(forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.READY_ALGAE)
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.setVelocity(0.0))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS));

    stateTriggers
        .get(SuperState.READY_ALGAE)
        .and(preClimbReq)
        .onTrue(forceState(SuperState.SPIT_ALGAE));

    stateTriggers
        .get(SuperState.SPIT_ALGAE)
        .whileTrue(manipulator.setVelocity(-10))
        .and(() -> !manipulator.getFirstBeambreak())
        .and(preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));

    stateTriggers.get(SuperState.PRE_PROCESSOR).whileTrue(elevator.setExtension(0.0));

    stateTriggers
        .get(SuperState.PRE_NET)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.MAX_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(Rotation2d.fromDegrees(65.0))) // TODO: get correct angle
        .whileTrue(wrist.setTargetAngle(Rotation2d.fromDegrees(145.0))); // TODO: get correct angle
    stateTriggers
        .get(SuperState.PRE_NET)
        .or(stateTriggers.get(SuperState.PRE_PROCESSOR))
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE));

    stateTriggers
        .get(SuperState.SCORE_ALGAE)
        .whileTrue(manipulator.setVelocity(40))
        // TODO: USE ALGAE BEAMBREAK
        .and(manipulator::getFirstBeambreak)
        .onFalse(forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_CLIMB)
        // TODO: MAKE CLIMBER WORK
        .and(climbCanReq)
        .and(climbConfReq)
        .onTrue(forceState(SuperState.CLIMB));

    stateTriggers.get(SuperState.CLIMB);
    // TODO: MAKE CLIMBER WORK

  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = nextState;
        });
  }
}
