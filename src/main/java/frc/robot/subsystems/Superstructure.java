package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
  private final Supplier<AlgaeIntakeTarget> algaeIntakeTarget;
  private final Supplier<AlgaeScoreTarget> algaeScoreTarget;

  private final Trigger preScoreReq;
  private final Trigger scoreReq;

  private final Trigger groundIntakeCoralReq;
  private final Trigger hpIntakeCoralReq;

  private final Trigger intakeAlgaeReq;

  private final Trigger preClimbReq;
  private final Trigger climbConfReq;
  private final Trigger climbCancelReq;

  private final Trigger antiJamReq;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;
  private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  private final ManipulatorSubsystem manipulator;
  private final FunnelSubsystem funnel;
  private final ClimberSubsystem climber;
  // Intake would be included here, but is cut from cad as of rn

  public Superstructure(
      ElevatorSubsystem elevator,
      ManipulatorSubsystem manipulator,
      ShoulderSubsystem shoulder,
      WristSubsystem wrist,
      FunnelSubsystem funnel,
      ClimberSubsystem climber,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> chassisVel,
      Supplier<ReefTarget> reefTarget,
      Supplier<AlgaeIntakeTarget> algaeIntakeTarget,
      Supplier<AlgaeScoreTarget> algaeScoreTarget,
      Trigger scoreReq,
      Trigger preScoreReq,
      Trigger groundIntakeCoralReq,
      Trigger hpIntakeCoralReq,
      Trigger intakeAlgaeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCancelReq,
      Trigger antiJamReq) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.funnel = funnel;
    this.climber = climber;

    this.pose = pose;
    this.chassisVel = chassisVel;
    this.reefTarget = reefTarget;
    this.algaeIntakeTarget = algaeIntakeTarget;
    this.algaeScoreTarget = algaeScoreTarget;

    this.preScoreReq = preScoreReq;
    this.scoreReq = scoreReq;

    this.groundIntakeCoralReq = groundIntakeCoralReq;
    this.hpIntakeCoralReq = hpIntakeCoralReq;

    this.intakeAlgaeReq = intakeAlgaeReq;

    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCancelReq = climbCancelReq;

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
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS))
        .whileTrue(climber.setPosition(0.0))
        .whileTrue(funnel.setVoltage(0.0));

    // IDLE -> INTAKE_CORAL_{HP/GROUND}
    stateTriggers
        .get(SuperState.IDLE)
        .and(hpIntakeCoralReq)
        .onTrue(this.forceState(SuperState.INTAKE_CORAL_HP));

    stateTriggers
        .get(SuperState.IDLE)
        .and(groundIntakeCoralReq)
        .onTrue(this.forceState(SuperState.INTAKE_CORAL_GROUND));

    // IDLE -> INTAKE_ALGAE_{location}
    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeIntakeTarget.get() == AlgaeIntakeTarget.GROUND)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_GROUND));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeIntakeTarget.get() == AlgaeIntakeTarget.HIGH)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_HIGH));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeIntakeTarget.get() == AlgaeIntakeTarget.LOW)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_LOW));

    stateTriggers
        .get(SuperState.IDLE)
        .and(intakeAlgaeReq)
        .and(() -> algaeIntakeTarget.get() == AlgaeIntakeTarget.STACK)
        .onTrue(this.forceState(SuperState.INTAKE_ALGAE_STACK));
    // IDLE -> PRE_CLIMB
    stateTriggers
        .get(SuperState.IDLE)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.PRE_CLIMB));

    // IDLE -> ANTI_JAM
    stateTriggers.get(SuperState.IDLE).and(antiJamReq).onTrue(forceState(SuperState.ANTI_JAM));

    // INTAKE_CORAL_HP logic
    stateTriggers
        .get(SuperState.INTAKE_CORAL_HP)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.HP_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_HP_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_HP_POS))
        .whileTrue(manipulator.index())
        .whileTrue(funnel.setVoltage(6.0))
        .and(manipulator::getSecondBeambreak)
        .onTrue(this.forceState(SuperState.READY_CORAL));

    // No-op until intake becomes a thing
    stateTriggers.get(SuperState.INTAKE_CORAL_GROUND).onTrue(this.forceState(SuperState.IDLE));
    // INTAKE_CORAL_GROUND -> ANTI_JAM
    stateTriggers
        .get(SuperState.INTAKE_CORAL_GROUND)
        .and(antiJamReq)
        .onTrue(forceState(SuperState.ANTI_JAM));
    // READY_CORAL logic
    stateTriggers
        .get(SuperState.READY_CORAL)
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS))
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.index()); // keep indexing to make sure its chilling
    // READY_CORAL -> ANTI_JAM
    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(antiJamReq)
        .onTrue(forceState(SuperState.ANTI_JAM));
    // SPIT_CORAL logic + -> IDLE
    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .whileTrue(manipulator.setVelocity(10))
        .and(() -> !manipulator.getFirstBeambreak())
        .and(() -> !manipulator.getSecondBeambreak())
        .and(preClimbReq.negate())
        .onTrue(this.forceState(SuperState.IDLE));
    // SPIT_CORAL -> PRE_CLIMB
    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .and(() -> !manipulator.getFirstBeambreak())
        .and(() -> !manipulator.getSecondBeambreak())
        .and(preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));
    // READY_CORAL -> PRE_L{1-4}
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
    // READY_CORAL -> SPIT_CORAL
    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.SPIT_CORAL));
    // PRE_L{1-4} logic + -> SCORE_CORAL
    stateTriggers
        .get(SuperState.PRE_L1)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L1_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L1_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L1_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L2)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L2_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L3)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L3_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L4)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L4_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));
    // SCORE_CORAL -> IDLE
    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .whileTrue(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed))
        .and(() -> !manipulator.getSecondBeambreak())
        .onTrue(this.forceState(SuperState.IDLE));
    // ANTI_JAM logic
    stateTriggers
        .get(SuperState.ANTI_JAM)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS));
    // ANTI_JAM -> IDLE
    stateTriggers.get(SuperState.ANTI_JAM).and(antiJamReq).onFalse(forceState(SuperState.IDLE));
    // INTAKE_ALGAE_{location} -> READY_ALGAE
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_GROUND_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_INTAKE_ALGAE_GROUND_POS))
        .whileTrue(manipulator.intakeAlgae())
        .and(() -> manipulator.getVoltage() < 0.0)
        .onTrue(this.forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_LOW)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.INTAKE_ALGAE_LOW_EXTENSION))
        .whileTrue(manipulator.intakeAlgae())
        .and(() -> manipulator.getVoltage() < 0.0)
        .onTrue(this.forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_HIGH)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.INTAKE_ALGAE_HIGH_EXTENSION))
        .whileTrue(manipulator.intakeAlgae())
        .and(() -> manipulator.getVoltage() < 0.0)
        .onTrue(forceState(SuperState.READY_ALGAE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_STACK)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.INTAKE_ALGAE_STACK_EXTENSION))
        .whileTrue(manipulator.intakeAlgae())
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_STACK_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_INTAKE_ALGAE_GROUND_POS))
        .and(() -> manipulator.getVoltage() < 0.0)
        .onTrue(forceState(SuperState.READY_ALGAE));
    // READY_ALGAE logic
    stateTriggers
        .get(SuperState.READY_ALGAE)
        .whileTrue(elevator.setExtension(0.0))
        .whileTrue(manipulator.intakeAlgae())
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS));
    // READY_ALGAE -> PRE_NET
    stateTriggers
        .get(SuperState.READY_ALGAE)
        .and(preScoreReq)
        .and(() -> algaeScoreTarget.get() == AlgaeScoreTarget.NET)
        .onTrue(forceState(SuperState.PRE_NET));
    // READY_ALGAE -> PRE_PROCESSOR
    stateTriggers
        .get(SuperState.READY_ALGAE)
        .and(preScoreReq)
        .and(() -> algaeScoreTarget.get() == AlgaeScoreTarget.PROCESSOR)
        .onTrue(forceState(SuperState.PRE_PROCESSOR));

    // READY_ALGAE -> SPIT_ALGAE
    stateTriggers
        .get(SuperState.READY_ALGAE)
        .and(preClimbReq)
        .onTrue(forceState(SuperState.SPIT_ALGAE));
    // SPIT_ALGAE -> PRE_CLIMB
    stateTriggers
        .get(SuperState.SPIT_ALGAE)
        // Positive bc algae is backwards
        .whileTrue(manipulator.setVelocity(10))
        .and(() -> !manipulator.getFirstBeambreak())
        .and(preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));
    // PRE_PROCESSOR logic
    stateTriggers.get(SuperState.PRE_PROCESSOR).whileTrue(elevator.setExtension(0.0));
    stateTriggers
        .get(SuperState.PRE_PROCESSOR)
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE));

    stateTriggers
        .get(SuperState.PRE_NET)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SHOOT_NET_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS));
    stateTriggers
        .get(SuperState.PRE_NET)
        .or(stateTriggers.get(SuperState.PRE_PROCESSOR))
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE));

    stateTriggers
        .get(SuperState.SCORE_ALGAE)
        .whileTrue(manipulator.setVelocity(40))
        .and(() -> manipulator.getVoltage() < 0.0)
        .and(manipulator::getFirstBeambreak)
        .onFalse(forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_CLIMB)
        // TODO: MAKE CLIMBER WORK
        .and(climbConfReq)
        .onTrue(forceState(SuperState.CLIMB));

    stateTriggers.get(SuperState.CLIMB);
    // TODO: MAKE CLIMBER WORK

    // May need more checks to see if canceling is safe
    stateTriggers.get(SuperState.CLIMB).and(climbCancelReq).onTrue(forceState(SuperState.IDLE));
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = nextState;
        });
  }
}
