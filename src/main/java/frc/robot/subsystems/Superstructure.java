package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem.ShoulderState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
import frc.robot.utils.FieldUtils.AlgaeIntakeTargets;
import frc.robot.utils.FieldUtils.L1Targets;
import frc.robot.utils.autoaim.AutoAim;

public class Superstructure {
   /**
   * We should have a state for every single "pose" the robot will hit. Hopefully we can get named
   * positions set up in cad to make this easier?
   */
  public enum SuperState {
    IDLE(ElevatorState.HP, ShoulderState.HP, WristState.HP, -7.0),
    PRE_INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ShoulderState.PRE_INTAKE_CORAL_GROUND,
        WristState.PRE_INTAKE_CORAL_GROUND,
        -18.0),
    INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ShoulderState.INTAKE_CORAL_GROUND,
        WristState.INTAKE_CORAL_GROUND,
        -18.0),
    POST_INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ShoulderState.PRE_INTAKE_CORAL_GROUND,
        WristState.PRE_INTAKE_CORAL_GROUND,
        0.0),
    READY_CORAL(ElevatorState.HP, ShoulderState.HP, WristState.HP, 0.0),
    PRE_L1(ElevatorState.L1, ShoulderState.PRE_L1, WristState.PRE_L1, 0.0),
    L1(ElevatorState.L1, ShoulderState.L1, WristState.PRE_L1, 3.0),
    POST_L1(ElevatorState.L1, ShoulderState.PRE_L1, WristState.PRE_L1, 0.0),
    PRE_L2(ElevatorState.L2, ShoulderState.PRE_L2, WristState.PRE_L2, 0.0),
    L2(ElevatorState.L2, ShoulderState.L2, WristState.L2, -15.0),
    POST_L2(ElevatorState.L2, ShoulderState.PRE_L2, WristState.PRE_L2, 0.0),
    PRE_L3(ElevatorState.L3, ShoulderState.PRE_L3, WristState.PRE_L3, 0.0),
    L3(ElevatorState.L3, ShoulderState.L3, WristState.L3, -15.0),
    POST_L3(ElevatorState.L3, ShoulderState.PRE_L3, WristState.PRE_L3, 0.0),
    PRE_PRE_L4(ElevatorState.HP, ShoulderState.PRE_L4, WristState.L4, 0.0),
    PRE_L4(
        ElevatorState.HP, ShoulderState.PRE_L4, WristState.L4, 0.0), // worried about shoulder here
    L4(ElevatorState.L4, ShoulderState.L4, WristState.L4, 20.0),
    POST_L4(ElevatorState.L4, ShoulderState.L4, WristState.HP, 0.0),
    POST_POST_L4(
        ElevatorState.HP, ShoulderState.L4, WristState.HP, 0.0), // like do we see the vision
    PRE_PRE_INTAKE_ALGAE(
        ElevatorState.HP,
        ShoulderState.PRE_INTAKE_ALGAE_REEF,
        WristState.PRE_INTAKE_ALGAE_REEF,
        ManipulatorSubsystem.MAX_VELOCITY * 10.0 / 12.0),
    PRE_INTAKE_ALGAE(
        ElevatorState.HP,
        ShoulderState.INTAKE_ALGAE_REEF,
        WristState.INTAKE_ALGAE_REEF,
        ManipulatorSubsystem.MAX_VELOCITY * 10.0 / 12.0),
    INTAKE_ALGAE_HIGH(
        ElevatorState.INTAKE_ALGAE_HIGH,
        ShoulderState.INTAKE_ALGAE_REEF,
        WristState.INTAKE_ALGAE_REEF,
        ManipulatorSubsystem.MAX_VELOCITY * 10.0 / 12.0),
    INTAKE_ALGAE_LOW(
        ElevatorState.INTAKE_ALGAE_LOW,
        ShoulderState.INTAKE_ALGAE_REEF,
        WristState.INTAKE_ALGAE_REEF,
        ManipulatorSubsystem.MAX_VELOCITY * 10.0 / 12.0),
    INTAKE_ALGAE_STACK(
        ElevatorState.INTAKE_ALGAE_STACK,
        ShoulderState.INTAKE_ALGAE_STACK,
        WristState.INTAKE_ALGAE_STACK,
        ManipulatorSubsystem.MAX_VELOCITY * 10.0 / 12.0),
    INTAKE_ALGAE_GROUND(
        ElevatorState.INTAKE_ALGAE_GROUND,
        ShoulderState.INTAKE_ALGAE_GROUND,
        WristState.INTAKE_ALGAE_GROUND,
        ManipulatorSubsystem.MAX_VELOCITY), // ?? lmao
    READY_ALGAE(
        ElevatorState.HP,
        ShoulderState.READY_ALGAE,
        WristState.READY_ALGAE,
        ManipulatorSubsystem.MAX_VELOCITY * 1.0 / 12.0),
    PRE_BARGE(
        ElevatorState.BARGE,
        ShoulderState.PRE_BARGE,
        WristState.PRE_BARGE,
        ManipulatorSubsystem.MAX_VELOCITY * 3.0 / 12.0),
    BARGE(
        ElevatorState.BARGE,
        ShoulderState.SCORE_BARGE,
        WristState.SCORE_BARGE,
        ManipulatorSubsystem.MAX_VELOCITY * -13.0 / 12.0), // TODO HELLO???
    POST_BARGE(ElevatorState.BARGE, ShoulderState.READY_ALGAE, WristState.PRE_BARGE, 0.0),
    POST_POST_BARGE(ElevatorState.HP, ShoulderState.PRE_BARGE, WristState.PRE_BARGE, 0.0),

    // SPIT_ALGAE,
    PROCESSOR(
        ElevatorState.PROCESSOR,
        ShoulderState.PROCESSOR,
        WristState.PROCESSOR,
        ManipulatorSubsystem.MAX_VELOCITY * -2.0 / 12.0),
    PRE_CLIMB(
        ElevatorState.INTAKE_ALGAE_GROUND,
        ShoulderState.INTAKE_ALGAE_GROUND,
        WristState.INTAKE_ALGAE_GROUND,
        0.0,
        3.4,
        2.0),
    CLIMB(
        ElevatorState.INTAKE_ALGAE_GROUND,
        ShoulderState.INTAKE_ALGAE_GROUND,
        WristState.INTAKE_ALGAE_GROUND,
        0.0,
        1.35,
        0.5) // lowkey why is this so slow
  // HOME(),
  // SPIT_CORAL(),
  // ANTI_JAM,
  // L4_TUCKED(ElevatorState.HP, ShoulderState.L4_TUCKED, WristState.L4_TUCKED),
  // L4_TUCKED_EXTENDED(ElevatorState.L4, ShoulderState.L4_TUCKED, WristState.L4_TUCKED),
  // L4_TUCKED_OUT(ElevatorState.L4, ShoulderState.L4_TUCKED_OUT, WristState.L4_TUCKED_OUT),

  ;

    public final ElevatorState elevatorState;
    public final ShoulderState shoulderState;
    public final WristState wristState;
    public final double manipulatorVelocity;
    public final double climberPosition;
    public final double climberSpeed;

    private SuperState(
        ElevatorState elevatorState,
        ShoulderState shoulderState,
        WristState wristState,
        double manipulatorVelocity) {
      this.elevatorState = elevatorState;
      this.shoulderState = shoulderState;
      this.wristState = wristState;
      this.manipulatorVelocity = manipulatorVelocity;
      this.climberPosition = 0.0; // TODO is this right? lol
      this.climberSpeed = 0.0;
    }

    private SuperState(
        ElevatorState elevatorState,
        ShoulderState shoulderState,
        WristState wristState,
        double manipulatorVelocity,
        double climberPosition,
        double climberSpeed) {
      this.elevatorState = elevatorState;
      this.shoulderState = shoulderState;
      this.wristState = wristState;
      this.manipulatorVelocity = manipulatorVelocity;
      this.climberPosition = climberPosition;
      this.climberSpeed = climberSpeed;
    }
  }

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  private final ManipulatorSubsystem manipulator;
  private final FunnelSubsystem funnel;
  private final ClimberSubsystem climber;
  private final SwerveSubsystem swerve;

    /** Creates a new Superstructure. */
    public Superstructure(
        ElevatorSubsystem elevator,
        ShoulderSubsystem shoulder,
        WristSubsystem wrist,
        ManipulatorSubsystem manipulator,
        FunnelSubsystem funnel,
        ClimberSubsystem climber,
        SwerveSubsystem swerve) {
      this.elevator = elevator;
      this.shoulder = shoulder;
      this.wrist = wrist;
      this.manipulator = manipulator;
      this.funnel = funnel;
      this.climber = climber;
      this.swerve = swerve;
  
      addTransitions();
  
      stateTimer.start();
    }

  /** This file is not a subsystem, so this MUST be called manually. */
  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
    Logger.recordOutput("Superstructure/State Timer", stateTimer.get());
  }

    /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger) {
    // maps triggers to the transitions
    trigger.and(new Trigger(() -> state == start)).onTrue(changeStateTo(end));
  }

  private boolean atExtension(SuperState state) {
    return elevator.atExtension(state.elevatorState.getExtensionMeters())
        && shoulder.isNearAngle(state.shoulderState.getAngle())
        && wrist.isNearAngle(state.wristState.getAngle());
  }

  public boolean atExtension() {
    return atExtension(state);
  }

  private Command changeStateTo(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
              stateTimer.reset();
              this.prevState = this.state;
              this.state = nextState;
              setSubstates();
            })
        .ignoringDisable(true);
  }

  public Command changeStateTo(Supplier<SuperState> state) {
    return changeStateTo(state.get());
  }

  private void setSubstates() {
    elevator.setState(state.elevatorState);
    shoulder.setState(state.shoulderState);
    wrist.setState(state.wristState);
    manipulator.setState(state.manipulatorVelocity);
    //funnel and climber don't have states per se
  }


  private void addTransitions() {
    // Prob a better way to impl this
    // Vaughn says he wants this available anytime
    //TODO this will probably not still work
    Robot.forceIndexReq.whileTrue(manipulator.setRollerVelocity(1.0));

    // ---Funnel---
    bindTransition(
        SuperState.IDLE,
        SuperState.READY_CORAL,
        new Trigger(() -> manipulator.getFirstBeambreak() && manipulator.getTimeSinceZero() < 1.0));

    // ---Intake coral ground---
    bindTransition(SuperState.IDLE, SuperState.PRE_INTAKE_CORAL_GROUND, Robot.intakeCoralReq);

    bindTransition(
        SuperState.PRE_INTAKE_CORAL_GROUND,
        SuperState.INTAKE_CORAL_GROUND,
        new Trigger(this::atExtension));

    bindTransition(
        SuperState.INTAKE_CORAL_GROUND,
        SuperState.POST_INTAKE_CORAL_GROUND,
        Robot.intakeCoralReq
            .negate()
            .debounce(0.060)
            .and(manipulator::bothBeambreaks)
            .debounce(0.12)); // TODO i'm lowkey losing my shit

    bindTransition(
        SuperState.POST_INTAKE_CORAL_GROUND,
        SuperState.READY_CORAL,
        new Trigger(this::atExtension));

    // ---L1---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L1,
        new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L1).and(Robot.preScoreReq));

    bindTransition(SuperState.PRE_L1, SuperState.L1, Robot.scoreReq.and(this::atExtension));
    
    //cancel
    bindTransition(SuperState.PRE_L1, SuperState.IDLE, new Trigger(() -> Robot.getCoralTarget() != ReefTarget.L1));

    bindTransition(
        SuperState.L1,
        SuperState.POST_L1,
        new Trigger(() -> manipulator.neitherBeambreak())
            .and(this::atExtension)
            .and(Robot.scoreReq.negate()));

    bindTransition(
        SuperState.POST_L1, 
        SuperState.IDLE, 
        Robot.intakeAlgaeReq.negate().or(() -> !intakeAlgaeFromReef()).and(this::atExtension).and(
        () ->
            L1Targets.getNearestLine(swerve.getPose()).getDistance(swerve.getPose().getTranslation()) > 0.3).debounce(0.15));

    //go straight to intaking algae from reef
    bindTransition(
        SuperState.POST_L1, 
        SuperState.PRE_PRE_INTAKE_ALGAE,
        new Trigger(this::atExtension)
         .and(Robot.intakeAlgaeReq)
         .and(() -> intakeAlgaeFromReef()));

    // ---L2---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L2,
        new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L2).and(Robot.preScoreReq));

    bindTransition(SuperState.PRE_L2, SuperState.L2, Robot.scoreReq.and(this::atExtension));

    //cancel
    bindTransition(SuperState.PRE_L2, SuperState.IDLE, new Trigger(() -> Robot.getCoralTarget() != ReefTarget.L2));

    bindTransition(
        SuperState.L2,
        SuperState.POST_L2,
        new Trigger(() -> manipulator.neitherBeambreak())
            .and(this::atExtension)
            .and(Robot.scoreReq.negate()));

    bindTransition(
        SuperState.POST_L2, 
        SuperState.IDLE, 
        Robot.intakeAlgaeReq.negate().or(() -> !intakeAlgaeFromReef()).and(this::atExtension).and(
        () ->
            L1Targets.getNearestLine(swerve.getPose()).getDistance(swerve.getPose().getTranslation()) > 0.3).debounce(0.15));

    //go straight to intaking algae from reef
    bindTransition(
        SuperState.POST_L2, 
        SuperState.PRE_PRE_INTAKE_ALGAE,
        new Trigger(this::atExtension)
         .and(Robot.intakeAlgaeReq)
         .and(() -> intakeAlgaeFromReef()));

    // ---L3---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L3,
        new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L3).and(Robot.preScoreReq));

    bindTransition(SuperState.PRE_L3, SuperState.L3, Robot.scoreReq.and(this::atExtension));

    //cancel
    bindTransition(SuperState.PRE_L3, SuperState.IDLE, new Trigger(() -> Robot.getCoralTarget() != ReefTarget.L3));

    bindTransition(
        SuperState.L3,
        SuperState.POST_L3,
        new Trigger(() -> manipulator.neitherBeambreak())
            .and(this::atExtension)
            .and(Robot.scoreReq.negate()));

    bindTransition(
        SuperState.POST_L3, 
        SuperState.IDLE, 
        Robot.intakeAlgaeReq.negate().or(() -> !intakeAlgaeFromReef()).and(this::atExtension).and(
        () ->
            L1Targets.getNearestLine(swerve.getPose()).getDistance(swerve.getPose().getTranslation()) > 0.3).debounce(0.15));

    //go straight to intaking algae from reef
    bindTransition(
        SuperState.POST_L3, 
        SuperState.PRE_PRE_INTAKE_ALGAE,
        new Trigger(this::atExtension)
         .and(Robot.intakeAlgaeReq)
         .and(() -> intakeAlgaeFromReef()));

    // ---L4---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_PRE_L4,
        new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L4).and(Robot.preScoreReq));

    bindTransition(SuperState.PRE_PRE_L4, SuperState.PRE_L4, new Trigger(this::atExtension));

    bindTransition(SuperState.PRE_L4, SuperState.L4, Robot.scoreReq.and(this::atExtension));

    //cancel
    bindTransition(SuperState.PRE_L4, SuperState.IDLE, new Trigger(() -> Robot.getCoralTarget() != ReefTarget.L4));

    bindTransition(
        SuperState.L4,
        SuperState.POST_L4,
        new Trigger(() -> manipulator.neitherBeambreak())
            .and(this::atExtension)
            .and(Robot.scoreReq.negate()));
    
    bindTransition(SuperState.POST_L4, SuperState.POST_POST_L4, new Trigger(this::atExtension));

    bindTransition(
        SuperState.POST_POST_L4,
        SuperState.IDLE, 
        Robot.intakeAlgaeReq.negate().or(() -> !intakeAlgaeFromReef()).and(this::atExtension).and(
        () ->
            L1Targets.getNearestLine(swerve.getPose()).getDistance(swerve.getPose().getTranslation()) > 0.3).debounce(0.15));

    //go straight to intaking algae from reef
    bindTransition(
        SuperState.POST_POST_L4,
        SuperState.PRE_PRE_INTAKE_ALGAE,
        new Trigger(this::atExtension)
         .and(Robot.intakeAlgaeReq)
         .and(() -> intakeAlgaeFromReef()));

    // ---Intake Algae---
    bindTransition(SuperState.IDLE, SuperState.PRE_PRE_INTAKE_ALGAE, Robot.intakeAlgaeReq);

    bindTransition(
        SuperState.PRE_PRE_INTAKE_ALGAE,
        SuperState.PRE_INTAKE_ALGAE,
        new Trigger(this::atExtension));

    // ---Intake Low Algae---
    bindTransition(
        SuperState.PRE_INTAKE_ALGAE,
        SuperState.INTAKE_ALGAE_LOW,
        new Trigger(this::atExtension)
            .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW)));

    //cancel
    bindTransition(
        SuperState.INTAKE_ALGAE_LOW,
        SuperState.IDLE,
        new Trigger(() -> Robot.getAlgaeIntakeTarget() != AlgaeIntakeTarget.LOW));

    // seems like the post pickup state is different for reef/ground?? why would you do this
    bindTransition(
        SuperState.INTAKE_ALGAE_LOW,
        SuperState.READY_ALGAE,
        new Trigger(() -> stateTimer.hasElapsed(1.0))
            .and(manipulator::hasAlgae)
            .and(() -> 
              AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose())
                                .getTranslation()
                                .minus(swerve.getPose().getTranslation())
                                .getNorm()
                            > 0.3
            ));

    // ---Intake High Algae---
    bindTransition(
        SuperState.PRE_INTAKE_ALGAE,
        SuperState.INTAKE_ALGAE_HIGH,
        new Trigger(this::atExtension)
            .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH)));

        //cancel
    bindTransition(
        SuperState.INTAKE_ALGAE_HIGH,
        SuperState.IDLE,
        new Trigger(() -> Robot.getAlgaeIntakeTarget() != AlgaeIntakeTarget.HIGH));

    // seems like the post pickup state is different for reef/ground?? why would you do this
    bindTransition(
        SuperState.INTAKE_ALGAE_HIGH,
        SuperState.READY_ALGAE,
        new Trigger(() -> stateTimer.hasElapsed(1.0))
            .and(manipulator::hasAlgae)
            .and(() -> 
              AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose())
                                .getTranslation()
                                .minus(swerve.getPose().getTranslation())
                                .getNorm()
                            > 0.3
            ));

    // ---Intake Stack Algae---
    bindTransition(
        SuperState.PRE_INTAKE_ALGAE,
        SuperState.INTAKE_ALGAE_STACK,
        new Trigger(this::atExtension)
            .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.STACK)));

        //cancel
    bindTransition(
        SuperState.INTAKE_ALGAE_STACK,
        SuperState.IDLE,
        new Trigger(() -> Robot.getAlgaeIntakeTarget() != AlgaeIntakeTarget.STACK));

    // seems like the post pickup state is different for reef/ground?? why would you do this
    bindTransition(
        SuperState.INTAKE_ALGAE_STACK,
        SuperState.READY_ALGAE,
        new Trigger(() -> stateTimer.hasElapsed(1.0))
            .and(manipulator::hasAlgae));

    // ---Intake Ground Algae---
    bindTransition(
        SuperState.PRE_INTAKE_ALGAE,
        SuperState.INTAKE_ALGAE_GROUND,
        new Trigger(this::atExtension)
            .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.GROUND)));

        //cancel
    bindTransition(
        SuperState.INTAKE_ALGAE_GROUND,
        SuperState.IDLE,
        new Trigger(() -> Robot.getAlgaeIntakeTarget() != AlgaeIntakeTarget.GROUND));

    // seems like the post pickup state is different for reef/ground?? why would you do this
    bindTransition(
        SuperState.INTAKE_ALGAE_GROUND,
        SuperState.READY_ALGAE,
        new Trigger(() -> stateTimer.hasElapsed(1.0))
            .and(manipulator::hasAlgae));

    // ---Score in barge---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_BARGE,
        new Trigger(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
            .and(Robot.preScoreReq));

    bindTransition(
        SuperState.PRE_BARGE, SuperState.BARGE, Robot.scoreReq.and(new Trigger(this::atExtension)));

    bindTransition(
        SuperState.BARGE,
        SuperState.POST_BARGE,
        new Trigger(() -> stateTimer.hasElapsed(0.5)) // TODO i don't trust this state timer stuff
        );

    bindTransition(
        SuperState.POST_BARGE,
        SuperState.POST_POST_BARGE,
        // new Trigger(() -> stateTimer.hasElapsed(1.0)));
        new Trigger(this::atExtension));

    bindTransition(SuperState.POST_POST_BARGE, SuperState.IDLE, new Trigger(this::atExtension));

    // ---Score in processor---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PROCESSOR,
        new Trigger(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.PROCESSOR)
            .and(Robot.preScoreReq));

    // TODO manipulator voltage gets set elsewhere i guess
    bindTransition(
        SuperState.PROCESSOR,
        SuperState.IDLE,
        new Trigger(
            () ->
                !MathUtil.isNear(
                        swerve.getPose().getX(),
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? AutoAim.BLUE_PROCESSOR_POS.getX()
                            : AutoAim.RED_PROCESSOR_POS.getX(),
                        0.5)
                    || !MathUtil.isNear(
                        swerve.getPose().getY(),
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? AutoAim.BLUE_PROCESSOR_POS.getY()
                            : AutoAim.RED_PROCESSOR_POS.getY(),
                        0.5))
        );

    stateTriggers
        .get(SuperState.IDLE)
        .and(() -> !elevator.hasZeroed || !wrist.hasZeroed)
        .and(() -> DriverStation.isEnabled())
        // .and(() -> Robot.ROBOT_TYPE != RobotType.SIM)
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
                shoulder.setAngle(Rotation2d.fromDegrees(50.0)),
                elevator.runCurrentZeroing(),
                Commands.waitUntil(() -> shoulder.getAngle().getDegrees() > 20.0)
                    .andThen(wrist.currentZero(() -> shoulder.getInputs()))))
        .and(() -> elevator.hasZeroed && wrist.hasZeroed && !homeReq.getAsBoolean())
        .onTrue(this.forceState(prevState));

    // SPIT_CORAL logic + -> IDLE
    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .whileTrue(manipulator.setRollerVelocity(10))
        .and(() -> !manipulator.getFirstBeambreak())
        .and(() -> !manipulator.getSecondBeambreak())
        .and(Robot.preClimbReq.negate())
        .onTrue(this.forceState(SuperState.IDLE));
    // SPIT_CORAL -> PRE_CLIMB
    stateTriggers
        .get(SuperState.SPIT_CORAL)
        .and(() -> !manipulator.getFirstBeambreak())
        .and(() -> !manipulator.getSecondBeambreak())
        .and(Robot.preClimbReq)
        .onTrue(forceState(SuperState.PRE_CLIMB));

    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(() -> !wrist.hasZeroed || !elevator.hasZeroed)
        .onTrue(this.forceState(SuperState.HOME));
    // READY_CORAL -> SPIT_CORAL
    stateTriggers
        .get(SuperState.READY_CORAL)
        .and(Robot.preClimbReq)
        .onTrue(this.forceState(SuperState.SPIT_CORAL));

    antiCoralJamReq
        .onTrue(this.forceState(SuperState.ANTI_CORAL_JAM))
        .onFalse(this.forceState(SuperState.IDLE));
    antiAlgaeJamReq
        .onTrue(this.forceState(SuperState.ANTI_ALGAE_JAM))
        .onFalse(this.forceState(SuperState.IDLE));
    // ANTI_JAM logic
    stateTriggers
        .get(SuperState.ANTI_CORAL_JAM)
        .whileTrue(elevator.hold())
        .whileTrue(wrist.hold())
        .whileTrue(shoulder.hold())
        .whileTrue(manipulator.setVelocity(-10))
        .whileTrue(funnel.setVoltage(-10.0));

    stateTriggers
        .get(SuperState.ANTI_ALGAE_JAM)
        .whileTrue(
            Commands.parallel(
                    elevator.hold(),
                    shoulder.setAngle(ShoulderSubsystem.SHOULDER_CORAL_GROUND_POS),
                    wrist.setAngle(WristSubsystem.WRIST_CORAL_GROUND))
                .until(() -> wrist.isNearTarget() && shoulder.getAngle().getDegrees() < 10.0)
                .andThen(
                    Commands.parallel(
                        wrist.hold(),
                        shoulder.hold(),
                        elevator.setExtension(Units.inchesToMeters(40)).andThen(elevator.hold()))));
    

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

        // ---Climb---
    // Climb could start from any state, so there's no particular transition
    Robot.preClimbReq.onTrue(
        Commands.parallel(changeStateTo(SuperState.PRE_CLIMB), funnel.unlatch()));

    bindTransition(SuperState.PRE_CLIMB, SuperState.CLIMB, Robot.climbConfReq);

    // May need more checks to see if canceling is safe
    bindTransition(SuperState.CLIMB, SuperState.PRE_CLIMB, Robot.climbCancelReq);
  }


  public SuperState getState() {
    return state;
  }

  public boolean intakeAlgaeFromReef() {
    return Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH
        || Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW;
  }

  public boolean stateIsCoralAlike() {
    return state == SuperState.READY_CORAL
        || state == SuperState.PRE_L1
        || state == SuperState.PRE_L2
        || state == SuperState.PRE_L3
        || state == SuperState.PRE_L4
        || state == SuperState.L1
        || state == SuperState.L2
        || state == SuperState.L3
        || state == SuperState.L4;
  }

  public boolean stateIsAlgaeAlike() {
    return state == SuperState.PRE_PRE_INTAKE_ALGAE
    || state == SuperState.PRE_INTAKE_ALGAE
    || state == SuperState.INTAKE_ALGAE_HIGH
    || state == SuperState.INTAKE_ALGAE_LOW
    || state == SuperState.INTAKE_ALGAE_STACK
    || state == SuperState.INTAKE_ALGAE_GROUND
    || state == SuperState.READY_ALGAE
    || state == SuperState.PRE_BARGE
    || state == SuperState.BARGE
    || state == SuperState.POST_BARGE
    || state == SuperState.POST_POST_BARGE
    || state == SuperState.PROCESSOR;
        // SPIT_ALGAE,
  }
}
