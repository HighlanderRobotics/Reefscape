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
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
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
  private AlgaeIntakeTarget prevAlgaeIntakeTarget;
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
      Trigger homeReq) {
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
    prevAlgaeIntakeTarget = algaeIntakeTarget.get();
    this.algaeScoreTarget = algaeScoreTarget;

    this.preScoreReq = preScoreReq;
    this.scoreReq = scoreReq;

    this.intakeAlgaeReq = intakeAlgaeReq;

    this.preClimbReq = climbReq;
    this.climbConfReq = climbConfReq;
    this.climbCancelReq = climbCancelReq;

    this.antiJamReq = antiJamReq;

    this.homeReq = homeReq;

    stateTimer.start();

    for (var state : SuperState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
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
        .whileTrue(elevator.setExtension(ElevatorSubsystem.HP_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_HP_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_HP_POS))
        .whileTrue(manipulator.index())
        .whileTrue(funnel.setVoltage(5.0))
        .and(manipulator::getSecondBeambreak)
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
                .andThen(this.forceState(SuperState.IDLE)))
        .and(() -> (elevator.hasZeroed && wrist.hasZeroed))
        .onTrue(this.forceState(SuperState.IDLE));

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
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L1_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L2)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L2_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L2_EXTENSION_METERS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L2_POS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L3)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L3_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L3_POS))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L3_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.PRE_L4)
        .whileTrue(elevator.setExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SCORE_L4_POS))
        .whileTrue(manipulator.setVelocity(0.0))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SCORE_POS))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SCORE_L4_POS))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.L4_EXTENSION_METERS))
        .and(scoreReq)
        .onTrue(this.forceState(SuperState.SCORE_CORAL));

    stateTriggers
        .get(SuperState.CHECK_ALGAE)
        .and(() -> stateTimer.hasElapsed(1.0))
        .and(() -> manipulator.getStatorCurrentAmps() <= 20.0)
        .onTrue(this.forceState(SuperState.IDLE));

    stateTriggers
        .get(SuperState.INTAKE_ALGAE_GROUND)
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_LOW))
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_HIGH))
        .or(stateTriggers.get(SuperState.INTAKE_ALGAE_STACK))
        .and(
            () -> {
              var diff = prevAlgaeIntakeTarget != algaeIntakeTarget.get();
              // This is ugly but we need to enforce update order
              prevAlgaeIntakeTarget = algaeIntakeTarget.get();
              return diff;
            })
        .onTrue(this.forceState(SuperState.IDLE));
    // SCORE_CORAL -> IDLE
    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .whileTrue(elevator.setExtension(() -> reefTarget.get().elevatorHeight))
        .whileTrue(wrist.setTargetAngle(() -> reefTarget.get().wristAngle))
        .whileTrue(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed))
        .whileTrue(
            Commands.parallel(
                shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                wrist.setTargetAngle(reefTarget.get().wristAngle)))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS))
        .debounce(0.25)
        .whileTrue(elevator.setExtension(0))
        .and(() -> elevator.isNearExtension(0))
        .and(() -> !manipulator.getSecondBeambreak())
        .onTrue(this.forceState(SuperState.IDLE));
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
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .whileTrue(manipulator.setVoltage(2 * ManipulatorSubsystem.ALGAE_HOLDING_VOLTAGE))
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                        shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                        wrist.setTargetAngle(WristSubsystem.WRIST_HP_POS))
                    .until(() -> elevator.isNearExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION)),
                Commands.parallel(
                        shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                        wrist.setSlowTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS))
                    .until(() -> wrist.isNearAngle(WristSubsystem.WRIST_SHOOT_NET_POS)),
                Commands.parallel(
                    shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SHOOT_NET_POS),
                    wrist.setSlowTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS))))
        .and(() -> wrist.isNearAngle(WristSubsystem.WRIST_SHOOT_NET_POS))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_SHOOT_NET_POS))
        .and(() -> elevator.isNearExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .and(scoreReq)
        .onTrue(forceState(SuperState.SCORE_ALGAE_NET));

    stateTriggers
        .get(SuperState.SCORE_ALGAE_NET)
        .onTrue(Commands.runOnce(() -> stateTimer.reset()))
        .whileTrue(manipulator.setVoltage(-ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE))
        .whileTrue(elevator.setExtension(ElevatorSubsystem.ALGAE_NET_EXTENSION))
        .whileTrue(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_SHOOT_NET_POS))
        .whileTrue(wrist.setTargetAngle(WristSubsystem.WRIST_SHOOT_NET_POS))
        .and(() -> stateTimer.hasElapsed(1))
        .whileTrue(
            Commands.parallel(
                shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS),
                wrist.setTargetAngle(Rotation2d.kZero)))
        .and(() -> shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS))
        .whileTrue(elevator.setExtension(0))
        .and(() -> elevator.isNearExtension(0))
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
            climber.setPosition(0.0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // May need more checks to see if canceling is safe
    stateTriggers
        .get(SuperState.CLIMB)
        .and(climbCancelReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));
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
