package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.autoaim.AlgaeIntakeTargets;
import frc.robot.utils.autoaim.HumanPlayerTargets;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum SuperState {
    IDLE,
    HOME,
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
    CHECK_ALGAE,
    READY_ALGAE,
    SPIT_ALGAE,
    PRE_PROCESSOR,
    PRE_NET,
    SCORE_ALGAE_NET,
    SCORE_ALGAE_PROCESSOR,
    PRE_CLIMB,
    CLIMB
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> chassisVel;
  private final Supplier<ReefTarget> reefTarget;
  private final Supplier<AlgaeIntakeTarget> algaeIntakeTarget;
  private final Supplier<AlgaeScoreTarget> algaeScoreTarget;

  @AutoLogOutput(key = "Superstructure/Pre Score Request")
  private final Trigger preScoreReq;

  @AutoLogOutput(key = "Superstructure/Score Request")
  private final Trigger scoreReq;

  @AutoLogOutput(key = "Superstructure/Algae Intake Request")
  private final Trigger intakeAlgaeReq;

  @AutoLogOutput(key = "Superstructure/Pre Climb Request")
  private final Trigger preClimbReq;

  @AutoLogOutput(key = "Superstructure/Climb Confirm Request")
  private final Trigger climbConfReq;

  @AutoLogOutput(key = "Superstructure/Climb Cancel Request")
  private final Trigger climbCancelReq;

  @AutoLogOutput(key = "Superstructure/Anti Jam Request")
  private final Trigger antiJamReq;

  @AutoLogOutput(key = "Superstructure/Home Request")
  private final Trigger homeReq;

  @AutoLogOutput(key = "Superstructure/Rev Funnel Req")
  private final Trigger revFunnelReq;

  @AutoLogOutput(key = "Superstructure/Force Funnel Req")
  private final Trigger forceFunnelReq;

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
      Trigger intakeAlgaeReq,
      Trigger climbReq,
      Trigger climbConfReq,
      Trigger climbCancelReq,
      Trigger antiJamReq,
      Trigger homeReq,
      Trigger revFunnelReq,
      Trigger forceFunnelReq) {
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

    this.intakeAlgaeReq = intakeAlgaeReq;

    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCancelReq = climbCancelReq;

    this.antiJamReq = antiJamReq;

    this.homeReq = homeReq;

    this.revFunnelReq = revFunnelReq;
    this.forceFunnelReq = forceFunnelReq;

    stateTimer.start();

    for (var state : SuperState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    configureStateTransitionCommands();
  }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
  }

  private void configureStateTransitionCommands() {
    stateTriggers
        .get(SuperState.IDLE)
        .whileTrue(
            extendWithClearance(
                ElevatorSubsystem.HP_EXTENSION_METERS,
                ShoulderSubsystem.SHOULDER_HP_POS,
                WristSubsystem.WRIST_HP_POS))
        .whileTrue(manipulator.index())
        .whileTrue(
            funnel.setVoltage(
                () ->
                    revFunnelReq.getAsBoolean()
                        ? -5.0
                        : (forceFunnelReq.getAsBoolean()
                                || (Stream.of(HumanPlayerTargets.values())
                                        .map(
                                            (t) ->
                                                t.location
                                                    .minus(pose.get())
                                                    .getTranslation()
                                                    .getNorm())
                                        .min(Double::compare)
                                        .get()
                                    < 1.0)
                            ? 12.0
                            : 0.0)))
        .and(manipulator::getFirstBeambreak)
        .onTrue(this.forceState(SuperState.READY_CORAL));

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

    stateTriggers
        .get(SuperState.IDLE)
        .and(() -> !elevator.hasZeroed || !wrist.hasZeroed)
        .and(() -> DriverStation.isEnabled())
        .and(() -> Robot.ROBOT_TYPE != RobotType.SIM)
        .onTrue(this.forceState(SuperState.HOME));

    // We might want to make this work when we have a piece as well?
    stateTriggers
        .get(SuperState.IDLE)
        .and(homeReq)
        .onTrue(this.forceState(SuperState.HOME))
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevator.hasZeroed = false;
                  wrist.hasZeroed = false;
                }));

    stateTriggers
        .get(SuperState.HOME)
        .whileTrue(
            Commands.parallel(
                    elevator.runCurrentZeroing(), wrist.currentZero(() -> shoulder.getInputs()))
                .andThen(Commands.waitUntil(homeReq.negate()), this.forceState(SuperState.IDLE)))
        .and(() -> elevator.hasZeroed && wrist.hasZeroed && !homeReq.getAsBoolean())
        .onTrue(this.forceState(prevState));

    // READY_CORAL logic
    stateTriggers
        .get(SuperState.READY_CORAL)
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS))
        .whileTrue(elevator.setExtension(ElevatorSubsystem.HP_EXTENSION_METERS))
        .whileTrue(manipulator.index());
    // keep indexing to make sure its chilling

    stateTriggers
        .get(SuperState.READY_CORAL)
        .or(stateTriggers.get(SuperState.PRE_L1))
        .or(stateTriggers.get(SuperState.PRE_L2))
        .or(stateTriggers.get(SuperState.PRE_L3))
        .or(stateTriggers.get(SuperState.PRE_L4))
        .and(() -> (!manipulator.getFirstBeambreak() && !manipulator.getSecondBeambreak()))
        .onTrue(this.forceState(SuperState.IDLE));

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

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(() -> !wrist.hasZeroed || !elevator.hasZeroed)
        .onTrue(this.forceState(SuperState.HOME));
    // READY_CORAL -> SPIT_CORAL
    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(preClimbReq)
        .onTrue(this.forceState(SuperState.SPIT_CORAL));
    // PRE_L{1-4} logic + -> SCORE_CORAL
    stateTriggers
        .get(SuperState.PRE_L1)
        .whileTrue(
            this.extendWithClearance(
                ElevatorSubsystem.L1_EXTENSION_METERS,
                ShoulderSubsystem.SHOULDER_SCORE_L1_POS,
                WristSubsystem.WRIST_SCORE_L1_POS))
        .whileTrue(manipulator.jog(1.4))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L1_EXTENSION_METERS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_L1_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L1_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L1)
        .and(() -> reefTarget.get() != ReefTarget.L1)
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_L2)
        .whileTrue(
            this.extendWithClearance(
                ElevatorSubsystem.L2_EXTENSION_METERS,
                ShoulderSubsystem.SHOULDER_SCORE_POS,
                WristSubsystem.WRIST_SCORE_L2_POS))
        .whileTrue(manipulator.jog(1.4))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L2_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L2)
        .and(() -> reefTarget.get() != ReefTarget.L2)
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_L3)
        .whileTrue(
            this.extendWithClearance(
                ElevatorSubsystem.L3_EXTENSION_METERS,
                ShoulderSubsystem.SHOULDER_SCORE_POS,
                WristSubsystem.WRIST_SCORE_L3_POS))
        .whileTrue(manipulator.jog(1.4))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L3_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L3)
        .and(() -> reefTarget.get() != ReefTarget.L3)
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_L4)
        .whileTrue(
            this.extendWithClearance(
                ElevatorSubsystem.L4_EXTENSION_METERS,
                ShoulderSubsystem.SHOULDER_SCORE_L4_POS,
                WristSubsystem.WRIST_SCORE_L4_POS))
        .whileTrue(manipulator.jog(1.4))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_L4_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L4_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L4)
        .and(() -> reefTarget.get() != ReefTarget.L4)
        .onTrue(this.forceState(SuperState.IDLE));

    // SCORE_CORAL -> IDLE
    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .whileTrue(elevator.setExtension(() -> reefTarget.get().elevatorHeight))
        .whileTrue(wrist.setTargetAngle(() -> reefTarget.get().wristAngle))
        .whileTrue(shoulder.setTargetAngle(() -> reefTarget.get().shoulderAngle))
        .whileTrue(manipulator.hold())
        .and(
            () ->
                shoulder.isNearAngle(reefTarget.get().shoulderAngle)
                    || (shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_WHACK_L1_POS)
                        && reefTarget.get() == ReefTarget.L1))
        .whileTrue(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed))
        .and(() -> reefTarget.get() == ReefTarget.L1)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L1_WHACK_CORAL_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_WHACK_L1_POS))
        .whileTrue(
            Commands.waitSeconds(0.1)
                .andThen(wrist.setTargetAngle(WristSubsystem.WRIST_WHACK_L1_POS)));

    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .and(() -> !manipulator.getFirstBeambreak() && !manipulator.getSecondBeambreak())
        .and(() -> !intakeAlgaeReq.getAsBoolean() || !intakeTargetOnReef())
        // .debounce(0.15)
        .onTrue(forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .and(() -> !manipulator.getFirstBeambreak() && !manipulator.getSecondBeambreak())
        .and(intakeAlgaeReq)
        .and(() -> intakeTargetOnReef())
        .onTrue(
            forceState(
                algaeIntakeTarget.get() == AlgaeIntakeTarget.HIGH
                    ? SuperState.INTAKE_ALGAE_HIGH
                    : SuperState.INTAKE_ALGAE_LOW));

    // ANTI_JAM logic
    stateTriggers
        .get(SuperState.ANTI_JAM)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .whileTrue(manipulator.setVelocity(10))
        .whileTrue(funnel.setVoltage(-10.0));

    stateTriggers
        .get(SuperState.CHECK_ALGAE)
        .and(() -> stateTimer.hasElapsed(1.0))
        .and(() -> manipulator.getStatorCurrentAmps() <= 20.0 && Robot.ROBOT_TYPE != RobotType.SIM)
        .onTrue(this.forceState(SuperState.IDLE));

    // change intake target
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .and(() -> algaeIntakeTarget.get() != AlgaeIntakeTarget.GROUND)
        .onTrue(this.forceState(SuperState.IDLE));
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_LOW)
        .and(() -> algaeIntakeTarget.get() != AlgaeIntakeTarget.LOW)
        .onTrue(this.forceState(SuperState.IDLE));
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_HIGH)
        .and(() -> algaeIntakeTarget.get() != AlgaeIntakeTarget.HIGH)
        .onTrue(this.forceState(SuperState.IDLE));
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_STACK)
        .and(() -> algaeIntakeTarget.get() != AlgaeIntakeTarget.STACK)
        .onTrue(this.forceState(SuperState.IDLE));

    // INTAKE_ALGAE_{location} -> READY_ALGAE
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.INTAKE_ALGAE_GROUND_EXTENSION))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_INTAKE_ALGAE_GROUND_POS))
        .whileTrue(
            Commands.waitUntil(
                    () -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_GROUND_POS))
                .andThen(manipulator.setVoltage(ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE)))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_GROUND_POS));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_LOW)
        .whileTrue(
            extendWithClearance(
                ElevatorSubsystem.INTAKE_ALGAE_LOW_EXTENSION,
                ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_POS,
                WristSubsystem.WRIST_INTAKE_ALGAE_REEF_POS))
        .whileTrue(manipulator.setVoltage(ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_HIGH)
        .whileTrue(
            extendWithClearance(
                ElevatorSubsystem.INTAKE_ALGAE_HIGH_EXTENSION,
                ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_POS,
                WristSubsystem.WRIST_INTAKE_ALGAE_REEF_POS))
        .whileTrue(manipulator.setVoltage(ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_STACK)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.INTAKE_ALGAE_STACK_EXTENSION))
        .whileTrue(
            Commands.waitUntil(
                    () -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_STACK_POS))
                .andThen(manipulator.setVoltage(-12.0)))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_STACK_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_INTAKE_ALGAE_STACK_POS))
        .and(
            () ->
                Robot.ROBOT_TYPE == RobotType.REAL
                    ? manipulator.getStatorCurrentAmps()
                        > ManipulatorSubsystem.ALGAE_CURRENT_THRESHOLD
                    : manipulator.hasAlgae())
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.INTAKE_ALGAE_STACK_EXTENSION))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_INTAKE_ALGAE_STACK_POS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_STACK_POS))
        .onTrue(forceState(SuperState.CHECK_ALGAE));

    // leave intake
    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_LOW))
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_HIGH))
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_STACK))
        .and(intakeAlgaeReq.negate())
        .onTrue(this.forceState(SuperState.CHECK_ALGAE));

    stateTriggers
        .get(SuperState.CHECK_ALGAE)
        .whileTrue(elevator.hold())
        .whileTrue(manipulator.intakeAlgae())
        .whileTrue(
            shoulder.setTargetAngle(
                () ->
                    (prevState == SuperState.INTAKE_ALGAE_HIGH
                            || prevState == SuperState.INTAKE_ALGAE_LOW)
                        ? ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_RETRACT_POS
                        : ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(
            wrist.setTargetAngle(
                () ->
                    (algaeIntakeTarget.get() == AlgaeIntakeTarget.GROUND
                            || algaeIntakeTarget.get() == AlgaeIntakeTarget.STACK)
                        ? WristSubsystem.WRIST_RETRACTED_POS
                        : WristSubsystem.WRIST_INTAKE_ALGAE_REEF_RETRACT_POS))
        .and(() -> stateTimer.hasElapsed(1.0))
        .and(() -> manipulator.getStatorCurrentAmps() > 20.0 || Robot.ROBOT_TYPE == RobotType.SIM)
        .and(
            () ->
                AlgaeIntakeTargets.getClosestTargetPose(pose.get())
                            .getTranslation()
                            .minus(pose.get().getTranslation())
                            .getNorm()
                        > 0.3
                    || (algaeIntakeTarget.get() == AlgaeIntakeTarget.GROUND
                        || algaeIntakeTarget.get() == AlgaeIntakeTarget.STACK))
        .onTrue(this.forceState(SuperState.READY_ALGAE));

    // READY_ALGAE logic
    stateTriggers
        .get(SuperState.READY_ALGAE)
        .whileTrue(
            extendWithClearance(
                0.0, ShoulderSubsystem.SHOULDER_RETRACTED_POS, WristSubsystem.WRIST_RETRACTED_POS))
        .whileTrue(manipulator.intakeAlgae());
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

    stateTriggers
        .get(SuperState.READY_ALGAE)
        .and(() -> manipulator.getStatorCurrentAmps() < 20.0 && Robot.ROBOT_TYPE != RobotType.SIM)
        .onTrue(forceState(SuperState.CHECK_ALGAE));
    // SPIT_ALGAE -> PRE_CLIMB
    stateTriggers
        .get(SuperState.SPIT_ALGAE)
        // Positive bc algae is backwards
        .whileTrue(manipulator.setVelocity(10))
        // Wait 1 second
        .and(() -> stateTimer.hasElapsed(1))
        .and(preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));

    // PRE_PROCESSOR logic
    stateTriggers
        .get(SuperState.PRE_PROCESSOR)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_PROCESSOR_EXTENSION))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_PROCESSOR_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_PROCESSOR_POS))
        .whileTrue(manipulator.setVoltage(ManipulatorSubsystem.ALGAE_HOLDING_VOLTAGE))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.ALGAE_PROCESSOR_EXTENSION))
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE_PROCESSOR));
    // PRE_NET logic
    stateTriggers
        .get(SuperState.PRE_NET)
        .whileTrue(manipulator.setVoltage(2 * ManipulatorSubsystem.ALGAE_HOLDING_VOLTAGE))
        .whileTrue(
            Commands.parallel(
                elevator.setExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION),
                shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_PRE_NET_POS),
                wrist.setTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS)))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SHOOT_NET_POS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_PRE_NET_POS))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE_NET));

    stateTriggers
        .get(SuperState.SCORE_ALGAE_NET)
        .onTrue(Commands.runOnce(() -> stateTimer.reset()))
        .whileTrue(manipulator.setVoltage(13.0))
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .whileTrue(shoulder.setTargetAngleSlow(ShoulderSubsystem.SHOULDER_SHOOT_NET_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS))
        .and(() -> stateTimer.hasElapsed(1))
        .onTrue(forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.SCORE_ALGAE_PROCESSOR)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_PROCESSOR_EXTENSION))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_RETRACTED_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_RETRACTED_POS))
        .whileTrue(manipulator.setVoltage(-ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE))
        .and(() -> stateTimer.hasElapsed(1.0))
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.PRE_CLIMB)
        .whileTrue(climber.setPosition(ClimberSubsystem.CLIMB_EXTENDED_POSITION))
        .onTrue(funnel.unlatch()) // !!
        .and(climbConfReq)
        .onTrue(forceState(SuperState.CLIMB));

    stateTriggers
        .get(SuperState.CLIMB)
        .whileTrue(
            climber.setPosition(1.3).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // May need more checks to see if canceling is safe
    stateTriggers
        .get(SuperState.CLIMB)
        .and(climbCancelReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));
  }

  private Command extendWithClearance(
      double elevatorExtension, Rotation2d shoulderAngle, Rotation2d wristAngle) {
    return extendWithClearance(() -> elevatorExtension, () -> shoulderAngle, () -> wristAngle);
  }

  private Command extendWithClearance(
      DoubleSupplier elevatorExtension,
      Supplier<Rotation2d> shoulderAngle,
      Supplier<Rotation2d> wristAngle) {
    return Commands.sequence(
        // Retract shoulder + wrist
        Commands.parallel(
                shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                wrist.setTargetAngle(WristSubsystem.WRIST_CLEARANCE_POS),
                elevator.hold())
            .until(
                () ->
                    shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS)
                        && wrist.isNearAngle(WristSubsystem.WRIST_CLEARANCE_POS)),
        // extend elevator
        Commands.parallel(
                shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                wrist.setTargetAngle(WristSubsystem.WRIST_CLEARANCE_POS),
                elevator.setExtension(elevatorExtension))
            .until(() -> elevator.isNearExtension(elevatorExtension.getAsDouble(), 0.05)),
        // re-extend joints
        Commands.parallel(
            shoulder.setTargetAngle(shoulderAngle),
            wrist.setTargetAngle(wristAngle),
            elevator.setExtension(elevatorExtension)));
  }

  public SuperState getState() {
    return state;
  }

  public boolean stateIsCoralAlike() {
    return this.state == SuperState.READY_CORAL
        || this.state == SuperState.PRE_L1
        || this.state == SuperState.PRE_L2
        || this.state == SuperState.PRE_L3
        || this.state == SuperState.PRE_L4
        || this.state == SuperState.SCORE_CORAL;
  }

  public boolean stateIsAlgaeAlike() {
    return this.state == SuperState.READY_ALGAE
        || this.state == SuperState.INTAKE_ALGAE_GROUND
        || this.state == SuperState.INTAKE_ALGAE_LOW
        || this.state == SuperState.INTAKE_ALGAE_HIGH
        || this.state == SuperState.INTAKE_ALGAE_STACK
        || this.state == SuperState.CHECK_ALGAE
        || this.state == SuperState.PRE_NET
        || this.state == SuperState.PRE_PROCESSOR
        || this.state == SuperState.SCORE_ALGAE_NET
        || this.state == SuperState.SCORE_ALGAE_PROCESSOR;
  }

  public boolean intakeTargetOnReef() {
    return this.algaeIntakeTarget.get() == AlgaeIntakeTarget.HIGH
        || this.algaeIntakeTarget.get() == AlgaeIntakeTarget.LOW;
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
              stateTimer.reset();
              this.prevState = this.state;
              this.state = nextState;
            })
        .ignoringDisable(true);
  }
}
