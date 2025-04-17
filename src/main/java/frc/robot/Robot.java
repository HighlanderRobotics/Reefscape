// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorSubsystem.ELEVATOR_ANGLE;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExtensionKinematics;
import frc.robot.subsystems.ExtensionKinematics.ExtensionState;
import frc.robot.subsystems.ExtensionPathing;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.servo.ServoIOReal;
import frc.robot.subsystems.shoulder.ShoulderIOReal;
import frc.robot.subsystems.shoulder.ShoulderIOSim;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.wrist.*;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.Tracer;
import frc.robot.utils.autoaim.AlgaeIntakeTargets;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.CageTargets;
import frc.robot.utils.autoaim.CoralTargets;
import frc.robot.utils.autoaim.L1Targets;
import java.util.HashMap;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotHardware {
    BANSHEE(new BansheeSwerveConstants()),
    ALPHA(new AlphaSwerveConstants()),
    KELPIE(new KelpieSwerveConstants());

    public final SwerveConstants swerveConstants;

    private RobotHardware(SwerveConstants swerveConstants) {
      this.swerveConstants = swerveConstants;
    }
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;
  // For replay to work properly this should match the hardware used in the log
  public static final RobotHardware ROBOT_HARDWARE = RobotHardware.KELPIE;
  // for testing class loading
  public static final ExtensionState test =
      ExtensionPathing.getNearest(new ExtensionState(0.0, Rotation2d.kZero, Rotation2d.kZero));

  public static enum ReefTarget {
    L1(
        ElevatorSubsystem.L1_EXTENSION_METERS,
        3.0,
        WristSubsystem.WRIST_SCORE_L1_POS,
        ShoulderSubsystem.SHOULDER_SCORE_POS),
    L2(
        ElevatorSubsystem.L2_EXTENSION_METERS,
        -15.0,
        WristSubsystem.WRIST_SCORE_L2_POS,
        ShoulderSubsystem.SHOULDER_SCORE_POS),
    L3(
        ElevatorSubsystem.L3_EXTENSION_METERS,
        -15.0,
        WristSubsystem.WRIST_SCORE_L3_POS,
        ShoulderSubsystem.SHOULDER_SCORE_POS),
    L4(
        ElevatorSubsystem.L4_EXTENSION_METERS,
        -20.0,
        WristSubsystem.WRIST_SCORE_L4_POS,
        ShoulderSubsystem.SHOULDER_SCORE_L4_POS);

    public final double elevatorHeight;
    public final double outtakeSpeed;
    public final Rotation2d wristAngle;
    public final Rotation2d shoulderAngle;

    private ReefTarget(
        double elevatorHeight,
        double outtakeSpeed,
        Rotation2d wristAngle,
        Rotation2d shoulderAngle) {
      this.elevatorHeight = elevatorHeight;
      this.outtakeSpeed = outtakeSpeed;
      this.wristAngle = wristAngle;
      this.shoulderAngle = shoulderAngle;
    }

    private ReefTarget(double elevatorHeight, Rotation2d wristAngle, Rotation2d shoulderAngle) {
      this.elevatorHeight = elevatorHeight;
      this.outtakeSpeed = 15.0;
      this.wristAngle = wristAngle;
      this.shoulderAngle = shoulderAngle;
    }
  }

  public static enum AlgaeIntakeTarget {
    LOW,
    HIGH,
    STACK,
    GROUND
  }

  public static enum AlgaeScoreTarget {
    NET,
    PROCESSOR
  }

  private static ReefTarget currentTarget = ReefTarget.L4;
  private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.NET;
  private boolean leftHandedTarget = false;

  @AutoLogOutput private boolean killVisionIK = true;

  @AutoLogOutput private boolean haveAutosGenerated = false;

  private static CANBus canivore = new CANBus("*");

  private static CANBusStatus canivoreStatus = canivore.getStatus();

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  // Create and configure a drivetrain simulation configuration
  private Optional<DriveTrainSimulationConfig> driveTrainSimulationConfig =
      ROBOT_TYPE == RobotType.SIM
          ? Optional.of(
              DriveTrainSimulationConfig.Default()
                  // Specify gyro type (for realistic gyro drifting and error simulation). i dont
                  // wanna
                  // deal w too much error lol
                  .withGyro(() -> new GyroSimulation(0.01, 0.01))
                  // Specify swerve module (for realistic swerve dynamics)
                  .withSwerveModule(
                      new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60Foc(1),
                          DCMotor.getKrakenX60Foc(1),
                          ROBOT_HARDWARE.swerveConstants.getDriveGearRatio(),
                          ROBOT_HARDWARE.swerveConstants.getTurnGearRatio(),
                          Volts.of(0.1),
                          Volts.of(0.2),
                          Meter.of(ROBOT_HARDWARE.swerveConstants.getWheelRadiusMeters()),
                          KilogramSquareMeters.of(0.03),
                          1.5))
                  // Configures the track length and track width (spacing between swerve modules)
                  .withTrackLengthTrackWidth(
                      Meter.of(ROBOT_HARDWARE.swerveConstants.getTrackWidthX()),
                      Meter.of(ROBOT_HARDWARE.swerveConstants.getTrackWidthY()))
                  // Configures the bumper size (dimensions of the robot bumper)
                  .withBumperSize(Inches.of(30), Inches.of(30))
                  .withRobotMass(ROBOT_HARDWARE.swerveConstants.getMass())
                  .withCustomModuleTranslations(
                      ROBOT_HARDWARE.swerveConstants.getModuleTranslations()))
          : Optional.empty();
  /* Create a swerve drive simulation */
  private Optional<SwerveDriveSimulation> swerveDriveSimulation =
      ROBOT_TYPE == RobotType.SIM
          ? Optional.of(
              new SwerveDriveSimulation(
                  // Specify Configuration
                  driveTrainSimulationConfig.get(),
                  // Specify starting pose
                  new Pose2d(3.28, 3.81, new Rotation2d())))
          : Optional.empty();

  private final SwerveSubsystem swerve =
      new SwerveSubsystem(
          ROBOT_HARDWARE.swerveConstants,
          ROBOT_TYPE != RobotType.SIM
              ? new GyroIOPigeon2(ROBOT_HARDWARE.swerveConstants.getGyroID())
              : new GyroIOSim(swerveDriveSimulation.get().getGyroSimulation()),
          Stream.of(ROBOT_HARDWARE.swerveConstants.getVisionConstants())
              .map(
                  (constants) ->
                      ROBOT_TYPE == RobotType.REAL
                          ? new VisionIOReal(constants)
                          : new VisionIOSim(constants))
              .toArray(VisionIO[]::new),
          ROBOT_TYPE != RobotType.SIM
              ? new ModuleIO[] {
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getFrontLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getFrontRightModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getBackLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getBackRightModule(),
                    ROBOT_HARDWARE.swerveConstants)
              }
              : new ModuleIO[] {
                new ModuleIOMapleSim(
                    ROBOT_HARDWARE.swerveConstants.getFrontLeftModule(),
                    ROBOT_HARDWARE.swerveConstants,
                    swerveDriveSimulation.get().getModules()[0]),
                new ModuleIOMapleSim(
                    ROBOT_HARDWARE.swerveConstants.getFrontRightModule(),
                    ROBOT_HARDWARE.swerveConstants,
                    swerveDriveSimulation.get().getModules()[1]),
                new ModuleIOMapleSim(
                    ROBOT_HARDWARE.swerveConstants.getBackLeftModule(),
                    ROBOT_HARDWARE.swerveConstants,
                    swerveDriveSimulation.get().getModules()[2]),
                new ModuleIOMapleSim(
                    ROBOT_HARDWARE.swerveConstants.getBackRightModule(),
                    ROBOT_HARDWARE.swerveConstants,
                    swerveDriveSimulation.get().getModules()[3])
              },
          PhoenixOdometryThread.getInstance(),
          swerveDriveSimulation,
          ROBOT_TYPE != RobotType.SIM
              ? new VisionIOReal(ROBOT_HARDWARE.swerveConstants.getAlgaeVisionConstants())
              : new VisionIOSim(ROBOT_HARDWARE.swerveConstants.getAlgaeVisionConstants()));

  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());

  private final ManipulatorSubsystem manipulator =
      new ManipulatorSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(
                  10,
                  RollerIOReal.getDefaultConfig()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withStatorCurrentLimitEnable(true)
                              .withStatorCurrentLimit(80.0)
                              .withSupplyCurrentLimit(30.0)
                              .withSupplyCurrentLimitEnable(true))
                      .withMotorOutput(
                          new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                      .withFeedback(
                          new FeedbackConfigs()
                              .withSensorToMechanismRatio(ManipulatorSubsystem.GEAR_RATIO))
                      .withSlot0(new Slot0Configs().withKV(0.928).withKP(0.5))
                      .withSlot1(new Slot1Configs().withKP(7.5).withKD(0.5).withKS(0.5)))
              : new RollerIOSim(
                  0.01,
                  5.8677,
                  new SimpleMotorFeedforward(0.0, 0.7),
                  new ProfiledPIDController(
                      0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 1))),
          new BeambreakIOReal(0, true),
          new BeambreakIOReal(1, true));

  private final ShoulderSubsystem shoulder =
      new ShoulderSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ShoulderIOReal() : new ShoulderIOSim());
  private final WristSubsystem wrist =
      new WristSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new WristIOReal(
                  12,
                  WristIOReal.getDefaultConfiguration()
                      .withSlot0(
                          new Slot0Configs().withKP(1000.0).withKD(30.0).withKS(0.3).withKV(3.2))
                      .withMotionMagic(WristSubsystem.DEFAULT_MOTION_MAGIC)
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withStatorCurrentLimit(50.0)
                              .withStatorCurrentLimitEnable(true)
                              .withSupplyCurrentLimit(20.0)
                              .withSupplyCurrentLimitEnable(true))
                      .withFeedback(
                          new FeedbackConfigs()
                              .withSensorToMechanismRatio(WristSubsystem.WRIST_GEAR_RATIO))
                      .withSoftwareLimitSwitch(
                          new SoftwareLimitSwitchConfigs()
                              .withForwardSoftLimitEnable(true)
                              .withForwardSoftLimitThreshold(0.5)))
              : new WristIOSim());

  private final FunnelSubsystem funnel =
      new FunnelSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(
                  19,
                  RollerIOReal.getDefaultConfig()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(10.0)
                              .withSupplyCurrentLimitEnable(true))
                      .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.0)))
              : new RollerIOSim(
                  0.01,
                  2.0,
                  new SimpleMotorFeedforward(0.0, 0.24),
                  new ProfiledPIDController(
                      1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(20, 10))),
          new ServoIOReal(0),
          new ServoIOReal(1));

  private final ClimberSubsystem climber =
      new ClimberSubsystem(ROBOT_TYPE != RobotType.SIM ? new ClimberIOReal() : new ClimberIOSim());

  private final Superstructure superstructure =
      new Superstructure(
          elevator,
          manipulator,
          shoulder,
          wrist,
          funnel,
          climber,
          swerve::getPose,
          swerve::getVelocityFieldRelative,
          () -> currentTarget,
          () -> algaeIntakeTarget,
          () -> algaeScoreTarget,
          driver
              .rightTrigger()
              .negate()
              .and(() -> DriverStation.isTeleop())
              .or(
                  new Trigger(
                          () -> {
                            final var state =
                                new ExtensionState(
                                    elevator.getExtensionMeters(),
                                    shoulder.getAngle(),
                                    wrist.getAngle());
                            final var branch =
                                ExtensionKinematics.getBranchPose(
                                    swerve.getPose(), state, currentTarget);
                            final var manipulatorPose =
                                ExtensionKinematics.getManipulatorPose(swerve.getPose(), state);
                            if (Robot.ROBOT_TYPE != RobotType.REAL)
                              Logger.recordOutput("IK/Manipulator Pose", manipulatorPose);
                            if (Robot.ROBOT_TYPE != RobotType.REAL)
                              Logger.recordOutput("IK/Branch", branch);
                            if (Robot.ROBOT_TYPE != RobotType.REAL)
                              Logger.recordOutput(
                                  "IK/Extension Check",
                                  manipulatorPose,
                                  manipulatorPose.transformBy(
                                      new Transform3d(
                                          Units.inchesToMeters(3.0), 0.0, 0.0, new Rotation3d())));
                            return false;
                            // return branch
                            //             .getTranslation()
                            //             .getDistance(manipulatorPose.getTranslation())
                            //         < Units.inchesToMeters(1.5)
                            //     || branch
                            //             .getTranslation()
                            //             .getDistance(
                            //                 manipulatorPose
                            //                     .transformBy(
                            //                         new Transform3d(
                            //                             Units.inchesToMeters(3.0),
                            //                             0.0,
                            //                             0.0,
                            //                             new Rotation3d()))
                            //                     .getTranslation())
                            //         < Units.inchesToMeters(1.5);
                          })
                      .debounce(0.15))
              //   .or(() -> AutoAim.isInToleranceCoral(swerve.getPose()))
              .or(() -> Autos.autoScore && DriverStation.isAutonomous())
              .or(
                  new Trigger(
                          () ->
                              AutoAim.isInToleranceCoral(
                                      swerve.getPose(),
                                      Units.inchesToMeters(1.5),
                                      Units.degreesToRadians(1.5))
                                  && MathUtil.isNear(
                                      0,
                                      Math.hypot(
                                          swerve.getVelocityRobotRelative().vxMetersPerSecond,
                                          swerve.getVelocityRobotRelative().vyMetersPerSecond),
                                      AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND)
                                  && MathUtil.isNear(
                                      0.0,
                                      swerve.getVelocityRobotRelative().omegaRadiansPerSecond,
                                      3.0)
                                  && currentTarget != ReefTarget.L4
                                  && currentTarget != ReefTarget.L1)
                      .debounce(0.08)
                      .and(() -> swerve.hasFrontTags)),
          driver
              .rightTrigger()
              .or(() -> Autos.autoPreScore && DriverStation.isAutonomous())
              .or(
                  () ->
                      swerve
                                  .getPose()
                                  .getTranslation()
                                  .minus(
                                      DriverStation.getAlliance().orElse(Alliance.Blue)
                                              == Alliance.Blue
                                          ? AutoAim.BLUE_REEF_CENTER
                                          : AutoAim.RED_REEF_CENTER)
                                  .getNorm()
                              < 3.25
                          && DriverStation.isAutonomous()),
          driver.leftTrigger().or(() -> Autos.autoAlgaeIntake && DriverStation.isAutonomous()),
          driver.leftBumper().or(() -> Autos.autoGroundCoralIntake && DriverStation.isAutonomous()),
          driver
              .x()
              .and(driver.pov(-1).negate())
              .debounce(0.5)
              .or(operator.x().and(operator.pov(-1).negate()).debounce(0.5)),
          driver.rightTrigger(),
          driver
              .y()
              .debounce(0.5)
              .or(operator.leftStick().and(operator.rightTrigger()).debounce(0.5)),
          driver.a(),
          driver.b(),
          driver.start(),
          operator.rightBumper(),
          operator.leftBumper(),
          operator.povDown(),
          new Trigger(() -> killVisionIK)
              .or(() -> currentTarget == ReefTarget.L1)
              .or(() -> DriverStation.isAutonomous()),
          () -> MathUtil.clamp(-operator.getLeftY(), -0.5, 0.5));

  private final LEDSubsystem leds = new LEDSubsystem(new LEDIOReal());

  private final Autos autos;
  private Optional<Alliance> lastAlliance = Optional.empty();
  // Could make this cache like Choreo's AutoChooser, but thats more work and Choreo's default
  // option isn't akit friendly
  // Main benefit to that is reducing startup time, which idt we care about too much
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  // Mechanisms
  private final LoggedMechanism2d elevatorMech2d =
      new LoggedMechanism2d(3.0, Units.feetToMeters(4.0));
  private final LoggedMechanismRoot2d
      elevatorRoot = // CAD distance from origin to center of carriage at full retraction
      elevatorMech2d.getRoot("Elevator", Units.inchesToMeters(21.5), 0.0);
  private final LoggedMechanismLigament2d carriageLigament =
      new LoggedMechanismLigament2d("Carriage", 0, ELEVATOR_ANGLE.getDegrees());
  private final LoggedMechanismLigament2d shoulderLigament =
      new LoggedMechanismLigament2d(
          "Arm", Units.inchesToMeters(15.7), ShoulderSubsystem.SHOULDER_RETRACTED_POS.getDegrees());
  private final LoggedMechanismLigament2d wristLigament =
      new LoggedMechanismLigament2d(
          "Wrist", Units.inchesToMeters(14.9), WristSubsystem.WRIST_RETRACTED_POS.getDegrees());

  public static Supplier<SuperState> state =
      () -> SuperState.IDLE; // TODO i feel like im breaking a rule

  @SuppressWarnings("resource")
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Comp2025");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_TYPE.toString());
    Logger.recordMetadata("Robot Hardware", ROBOT_HARDWARE.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    initializeTracerLogging();

    switch (ROBOT_TYPE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    swerve.startOdoThread();
    SignalLogger.setPath("/media/sda1/");
    Logger.recordOutput("Canivore Status", canivoreStatus.Status);

    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation.orElse(null));
      swerve.resetPose(swerveDriveSimulation.get().getSimulatedDriveTrainPose());
      // global static is mildly questionable
      VisionIOSim.pose = () -> new Pose3d(swerveDriveSimulation.get().getSimulatedDriveTrainPose());
    } else {
      // this should never be called?
      VisionIOSim.pose = () -> new Pose3d();
    }
    // Add the arms and stuff
    elevatorRoot.append(carriageLigament);
    carriageLigament.append(shoulderLigament);
    shoulderLigament.append(wristLigament);

    autos = new Autos(swerve, manipulator, funnel, elevator, shoulder, wrist);
    autoChooser.addDefaultOption("None", autos.getNoneAuto());

    SmartDashboard.putData(
        "Run Elevator Sysid",
        elevator
            .runSysid()
            .raceWith(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS)));

    SmartDashboard.putData(
        "Step Elevator Current",
        elevator
            .setCurrent(60.0)
            .raceWith(shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_CLEARANCE_POS)));

    SmartDashboard.putData(
        "Check Clear",
        Commands.parallel(
            shoulder.setTargetAngle(ShoulderSubsystem.SHOULDER_TUCKED_CLEARANCE_POS),
            wrist.setTargetAngle(WristSubsystem.WRIST_TUCKED_CLEARANCE_POS)));

    SmartDashboard.putData(
        "Manual Zero Extension",
        Commands.runOnce(
                () -> {
                  elevator.resetExtension(0.0);
                  wrist.resetPosition(Rotation2d.k180deg);
                })
            .ignoringDisable(true));

    System.out.println("Node Count " + ExtensionPathing.graph.nodes().size());

    SmartDashboard.putData(
        "Traverse Extension Graph",
        superstructure
            .extendWithClearance(
                () ->
                    new ExtensionState(
                        ElevatorSubsystem.HP_EXTENSION_METERS,
                        ShoulderSubsystem.SHOULDER_HP_POS,
                        WristSubsystem.WRIST_HP_POS))
            .until(
                () ->
                    elevator.isNearExtension(ElevatorSubsystem.HP_EXTENSION_METERS)
                        && shoulder.isNearAngle(ShoulderSubsystem.SHOULDER_HP_POS)
                        && wrist.isNearAngle(WristSubsystem.WRIST_HP_POS))
            .andThen(
                Commands.sequence(
                    ExtensionPathing.graph.nodes().stream()
                        .map(
                            (node) ->
                                superstructure
                                    .extendWithClearance(() -> node)
                                    .alongWith(
                                        Commands.print("Traversing to " + node),
                                        Commands.runOnce(
                                            () -> Logger.recordOutput("Traversal Target", node)))
                                    .until(
                                        () ->
                                            elevator.isNearExtension(node.elevatorHeightMeters())
                                                && shoulder.isNearAngle(node.shoulderAngle())
                                                && wrist.isNearAngle(node.wristAngle()))
                                    .finallyDo(() -> System.out.println("done"))
                                    .andThen(
                                        Commands.waitSeconds(0.5),
                                        superstructure
                                            .extendWithClearance(
                                                () ->
                                                    new ExtensionState(
                                                        ElevatorSubsystem.HP_EXTENSION_METERS,
                                                        ShoulderSubsystem.SHOULDER_HP_POS,
                                                        WristSubsystem.WRIST_HP_POS))
                                            .alongWith(Commands.print("Retracting"))
                                            .until(
                                                () ->
                                                    elevator.isNearExtension(
                                                            ElevatorSubsystem.HP_EXTENSION_METERS)
                                                        && shoulder.isNearAngle(
                                                            ShoulderSubsystem.SHOULDER_HP_POS)
                                                        && wrist.isNearAngle(
                                                            WristSubsystem.WRIST_HP_POS)),
                                        Commands.waitSeconds(0.5)))
                        .toArray(Command[]::new))));

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    // Default Commands

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    new Trigger(
            () ->
                superstructure.getState() == SuperState.READY_ALGAE
                    || superstructure.getState() == SuperState.READY_CORAL)
        .onTrue(driver.rumbleCmd(1.0, 1.0).withTimeout(0.5));

    new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop())
        .onTrue(Commands.runOnce(() -> Autos.autoScore = false));

    new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop())
        .onTrue(Commands.runOnce(() -> Autos.autoPreScore = false));

    new Trigger(() -> DriverStation.isEnabled() && DriverStation.isTeleop())
        .onTrue(Commands.runOnce(() -> Autos.autoGroundCoralIntake = false));

    new Trigger(() -> DriverStation.isAutonomousEnabled() && !wrist.hasZeroed)
        .onTrue(
            Commands.runOnce(
                () -> {
                  wrist.resetPosition(Rotation2d.fromRadians(3.094));
                  elevator.resetExtension(0.0);
                }));

    new Trigger(
            () -> {
              var allianceChange = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChange && DriverStation.getAlliance().isPresent();
            })
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(leds.setBlinkingCmd(Color.kWhite, Color.kBlack, 20.0).withTimeout(1.0))
                .ignoringDisable(true));

    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated)
        .onTrue(Commands.print("connected"))
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(leds.setBlinkingCmd(Color.kWhite, Color.kBlack, 20.0).withTimeout(1.0))
                .ignoringDisable(true));

    new Trigger(() -> DriverStation.isAutonomousEnabled())
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerve.setCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(false)
                                .withSupplyCurrentLimitEnable(false)))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () ->
                        swerve.setCurrentLimits(
                            ROBOT_HARDWARE.swerveConstants.getDriveConfig().CurrentLimits))
                .ignoringDisable(true));

    SmartDashboard.putData(
        "Add Autos",
        Commands.runOnce(
                () -> {
                  if (DriverStation.getAlliance().isPresent()) {
                    addAutos();
                  }
                })
            .ignoringDisable(true));
    elevator.setDefaultCommand(
        Commands.sequence(
                elevator.runCurrentZeroing().onlyIf(() -> !elevator.hasZeroed),
                elevator.setExtension(0.0).until(() -> elevator.isNearExtension(0.0)),
                elevator.setVoltage(0.0))
            .withName("Elevator Default Command"));

    manipulator.setDefaultCommand(manipulator.hold());

    shoulder.setDefaultCommand(shoulder.hold());

    wrist.setDefaultCommand(wrist.hold());

    funnel.setDefaultCommand(funnel.setVoltage(0.0));

    climber.setDefaultCommand(climber.setPosition(0.0));

    leds.setDefaultCommand(
        Commands.either(
                leds.setBlinkingCmd(
                        () -> LEDSubsystem.getReefTargetColor(currentTarget),
                        () ->
                            superstructure.getState() == SuperState.IDLE
                                ? Color.kBlack
                                : Color.kWhite,
                        5.0)
                    .until(() -> !DriverStation.isEnabled()),
                leds.setRunAlongCmd(
                        () ->
                            DriverStation.getAlliance()
                                .map((a) -> a == Alliance.Blue ? Color.kBlue : Color.kRed)
                                .orElse(Color.kWhite),
                        () -> wrist.hasZeroed ? LEDSubsystem.PURPLE : Color.kOrange,
                        4,
                        1.0)
                    .until(() -> DriverStation.isEnabled()),
                () -> DriverStation.isEnabled())
            .repeatedly()
            .ignoringDisable(true));

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                        modifyJoystick(driver.getLeftY())
                            * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                        modifyJoystick(driver.getLeftX())
                            * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                        modifyJoystick(driver.getRightX())
                            * ROBOT_HARDWARE.swerveConstants.getMaxAngularSpeed())
                    .times(-1)));

    driver
        .rightBumper()
        .or(driver.leftBumper())
        .and(() -> superstructure.stateIsCoralAlike() && currentTarget != ReefTarget.L1)
        .whileTrue(
            Commands.parallel(
                AutoAim.autoAimWithIntermediatePose(
                    swerve,
                    () -> {
                      var twist = swerve.getVelocityFieldRelative().toTwist2d(0.3);
                      return CoralTargets.getHandedClosestTarget(
                          swerve
                              .getPose()
                              .plus(
                                  new Transform2d(
                                      twist.dx, twist.dy, Rotation2d.fromRadians(twist.dtheta))),
                          driver.leftBumper().getAsBoolean());
                    },
                    // Keeps the robot off the reef wall until it's aligned side-side
                    new Transform2d(
                        AutoAim.INITIAL_REEF_KEEPOFF_DISTANCE_METERS, 0.0, Rotation2d.kZero)),
                Commands.waitUntil(() -> AutoAim.isInToleranceCoral(swerve.getPose()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .rightBumper()
        .or(driver.leftBumper())
        .and(() -> superstructure.stateIsCoralAlike() && currentTarget == ReefTarget.L1)
        .whileTrue(
            Commands.parallel(
                AutoAim.alignToLine(
                    swerve,
                    () ->
                        modifyJoystick(driver.getLeftY())
                            * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    () ->
                        modifyJoystick(driver.getLeftX())
                            * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    () -> L1Targets.getNearestLine(swerve.getPose())),
                Commands.waitUntil(
                        () ->
                            AutoAim.isInTolerance(
                                swerve.getPose(),
                                new Pose2d(
                                    L1Targets.getNearestLine(swerve.getPose())
                                        .nearest(swerve.getPose().getTranslation()),
                                    L1Targets.getNearestLine(swerve.getPose()).getRotation())))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .rightBumper()
        .and(
            () ->
                superstructure.getState() == SuperState.INTAKE_ALGAE_HIGH
                    || superstructure.getState() == SuperState.INTAKE_ALGAE_LOW
                    || superstructure.getState() == SuperState.INTAKE_ALGAE_STACK
                    || superstructure.getState() == SuperState.IDLE)
        .whileTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          algaeIntakeTarget =
                              AlgaeIntakeTargets.getClosestTarget(swerve.getPose()).height;
                        }),
                    AutoAim.translateToPose(
                            swerve,
                            () ->
                                AlgaeIntakeTargets.getOffsetLocation(
                                    AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose())))
                        .until(
                            () ->
                                AutoAim.isInTolerance(
                                        swerve.getPose(),
                                        AlgaeIntakeTargets.getOffsetLocation(
                                            AlgaeIntakeTargets.getClosestTargetPose(
                                                swerve.getPose())),
                                        swerve.getVelocityFieldRelative(),
                                        Units.inchesToMeters(1.0),
                                        Units.degreesToRadians(1.0))
                                    && elevator.isNearTarget()
                                    && shoulder.isNearAngle(
                                        ShoulderSubsystem.SHOULDER_INTAKE_ALGAE_REEF_POS)
                                    && wrist.isNearAngle(
                                        WristSubsystem.WRIST_INTAKE_ALGAE_REEF_POS)),
                    AutoAim.approachAlgae(
                        swerve,
                        () -> AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose()),
                        1)),
                Commands.waitUntil(
                        new Trigger(
                                () ->
                                    AutoAim.isInToleranceAlgaeIntake(swerve.getPose())
                                        && MathUtil.isNear(
                                            0,
                                            Math.hypot(
                                                swerve.getVelocityRobotRelative().vxMetersPerSecond,
                                                swerve.getVelocityRobotRelative()
                                                    .vyMetersPerSecond),
                                            AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND)
                                        && MathUtil.isNear(
                                            0.0,
                                            swerve.getVelocityRobotRelative().omegaRadiansPerSecond,
                                            3.0))
                            .debounce(0.08)
                            .and(() -> swerve.hasFrontTags))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .rightBumper()
        .or(driver.leftBumper())
        .and(() -> superstructure.getState() == SuperState.INTAKE_ALGAE_GROUND)
        .whileTrue(
            swerve.driveToAlgae(
                () ->
                    modifyJoystick(driver.getLeftY())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                () ->
                    modifyJoystick(driver.getLeftX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                () ->
                    modifyJoystick(driver.getRightX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxAngularSpeed()));
    driver
        .rightBumper()
        .or(driver.leftBumper())
        .and(
            () ->
                superstructure.getState() == SuperState.READY_ALGAE
                    || superstructure.getState() == SuperState.PRE_PROCESSOR)
        .and(() -> algaeScoreTarget == AlgaeScoreTarget.PROCESSOR)
        .whileTrue(
            Commands.parallel(
                AutoAim.autoAimWithIntermediatePose(
                    swerve,
                    () -> swerve.getPose().nearest(AutoAim.PROCESSOR_POSES),
                    new Transform2d(
                        -(ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2) - 0.5,
                        0.0,
                        Rotation2d.kZero)),
                Commands.waitUntil(
                        () ->
                            AutoAim.isInTolerance(
                                swerve
                                    .getPose()
                                    .nearest(AutoAim.PROCESSOR_POSES)
                                    // Moves the target pose inside the field, with the bumpers
                                    // aligned with the wall
                                    .transformBy(
                                        new Transform2d(
                                            -(ROBOT_HARDWARE.swerveConstants.getBumperLength() / 2),
                                            0.0,
                                            Rotation2d.kZero)),
                                swerve.getPose()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .rightBumper()
        .and(
            () ->
                superstructure.getState() == SuperState.PRE_CLIMB
                    || superstructure.getState() == SuperState.CLIMB)
        .whileTrue(
            Commands.parallel(
                AutoAim.translateToPose(
                    swerve, () -> CageTargets.getOffsetClosestTarget(swerve.getPose())),
                Commands.waitUntil(
                        () ->
                            AutoAim.isInTolerance(
                                CageTargets.getOffsetClosestTarget(swerve.getPose()),
                                swerve.getPose()))
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .rightBumper()
        .and(
            () ->
                superstructure.getState() == SuperState.READY_ALGAE
                    || superstructure.getState() == SuperState.PRE_NET)
        .and(() -> algaeScoreTarget == AlgaeScoreTarget.NET)
        .whileTrue(
            Commands.parallel(
                AutoAim.translateToXCoord(
                    swerve,
                    () ->
                        Math.abs(swerve.getPose().getX() - AutoAim.BLUE_NET_X)
                                > Math.abs(swerve.getPose().getX() - AutoAim.RED_NET_X)
                            ? AutoAim.RED_NET_X
                            : AutoAim.BLUE_NET_X,
                    () ->
                        modifyJoystick(driver.getLeftX())
                            * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    () ->
                        (Math.abs(swerve.getPose().getX() - AutoAim.BLUE_NET_X)
                                    > Math.abs(swerve.getPose().getX() - AutoAim.RED_NET_X)
                                ? Rotation2d.kZero
                                : Rotation2d.k180deg)
                            .plus(Rotation2d.fromDegrees(30.0))),
                Commands.waitUntil(
                        () -> {
                          final var diff =
                              swerve
                                  .getPose()
                                  .minus(AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose()));
                          return MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(1.0))
                              && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(1.0))
                              && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
                        })
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    driver
        .povUp()
        .and(() -> ROBOT_TYPE == RobotType.SIM)
        .onTrue(Commands.runOnce(() -> manipulator.setFirstBeambreak(true)).ignoringDisable(true));
    driver
        .povDown()
        .and(() -> ROBOT_TYPE == RobotType.SIM)
        .onTrue(Commands.runOnce(() -> manipulator.setFirstBeambreak(false)).ignoringDisable(true));
    driver
        .povRight()
        .and(() -> ROBOT_TYPE == RobotType.SIM)
        .onTrue(Commands.runOnce(() -> manipulator.setHasAlgae(!manipulator.hasAlgae())));

    RobotModeTriggers.autonomous()
        .and(() -> ROBOT_TYPE == RobotType.SIM)
        .onTrue(Commands.runOnce(() -> manipulator.setSecondBeambreak(true)).ignoringDisable(true));
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (ROBOT_TYPE == RobotType.SIM) {
                    swerveDriveSimulation.get().setSimulationWorldPose(swerve.getPose());
                  }
                }));
    driver.x().onTrue(Commands.runOnce(() -> shoulder.rezero()).ignoringDisable(true));

    operator
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentTarget = ReefTarget.L1;
                  algaeIntakeTarget = AlgaeIntakeTarget.GROUND;
                }));
    operator
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentTarget = ReefTarget.L2;
                  algaeIntakeTarget = AlgaeIntakeTarget.LOW;
                }));
    operator
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentTarget = ReefTarget.L3;
                  algaeIntakeTarget = AlgaeIntakeTarget.HIGH;
                }));
    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentTarget = ReefTarget.L4;
                  algaeIntakeTarget = AlgaeIntakeTarget.STACK;
                }));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> algaeScoreTarget = AlgaeScoreTarget.NET));

    operator
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> algaeScoreTarget = AlgaeScoreTarget.PROCESSOR));

    operator.povLeft().onTrue(Commands.runOnce(() -> leftHandedTarget = true));
    operator.povRight().onTrue(Commands.runOnce(() -> leftHandedTarget = false));

    operator
        .back()
        .and(operator.start())
        .onTrue(Commands.runOnce(() -> killVisionIK = !killVisionIK));

    new Trigger(() -> superstructure.stateIsAlgaeAlike())
        .whileTrue(
            leds.setBlinkingSplitCmd(
                () -> LEDSubsystem.getAlgaeIntakeTargetColor(algaeIntakeTarget),
                () ->
                    LEDSubsystem.getAlgaeScoringTargetColor(
                        algaeScoreTarget == AlgaeScoreTarget.NET),
                () -> Color.kBlack,
                5.0));
    // heading reset
    driver
        .leftStick()
        .and(driver.rightStick())
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().equals(Alliance.Blue)
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg)));

    // Log locations of all autoaim targets
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/Targets/Coral",
          Stream.of(CoralTargets.values())
              .map((target) -> CoralTargets.getRobotTargetLocation(target.location))
              .toArray(Pose2d[]::new));
    // Log locations of all autoaim targets
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/Targets/Cage",
          Stream.of(CageTargets.values())
              .map((target) -> target.getLocation())
              .toArray(Pose2d[]::new));
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/Targets/L1",
          Stream.of(L1Targets.values())
              .map((target) -> L1Targets.getRobotTargetLine(target.line))
              .toArray(Rectangle2d[]::new));
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/Targets/Algae",
          Stream.of(AlgaeIntakeTargets.values())
              .map((target) -> AlgaeIntakeTargets.getRobotTargetLocation(target.location))
              .toArray(Pose2d[]::new));

    Logger.recordOutput("IK/L1 FK Pose", ExtensionKinematics.L1_POSE);
    System.out.println("ExtensionKinematics.L1_POSE: " + ExtensionKinematics.L1_POSE);
    Logger.recordOutput("IK/L2 FK Pose", ExtensionKinematics.L2_POSE);
    System.out.println("ExtensionKinematics.L2_POSE: " + ExtensionKinematics.L2_POSE);
    Logger.recordOutput("IK/L3 FK Pose", ExtensionKinematics.L3_POSE);
    System.out.println("ExtensionKinematics.L3_POSE: " + ExtensionKinematics.L3_POSE);
    Logger.recordOutput("IK/L4 FK Pose", ExtensionKinematics.L4_POSE);
    System.out.println("ExtensionKinematics.L4_POSE: " + ExtensionKinematics.L4_POSE);
  }

  private void addAutos() {
    System.out.println("------- Regenerating Autos");
    System.out.println(
        "Regenerating Autos on " + DriverStation.getAlliance().map((a) -> a.toString()));
    autoChooser.addOption("Triangle Test", autos.getTestTriangle());
    autoChooser.addOption("Sprint Test", autos.getTestSprint());
    autoChooser.addOption("LM to H", autos.LMtoH());
    autoChooser.addOption("RM to G", autos.RMtoG());
    autoChooser.addOption("4.5 L Outside", autos.LOtoJ());
    autoChooser.addOption("4.5 R Outside", autos.ROtoE());
    autoChooser.addOption("4.5 L Inside", autos.LItoK());
    autoChooser.addOption("4.5 R Inside", autos.RItoD());
    autoChooser.addOption("Push Auto", autos.PMtoPL());
    autoChooser.addOption("Algae auto", autos.CMtoGH());
    autoChooser.addOption("!!! DO NOT RUN!! 2910 auto", autos.LOtoA());
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  @Override
  public void loopFunc() {
    Tracer.trace("Robot/LoopFunc", super::loopFunc);
  }

  private void initializeTracerLogging() {
    HashMap<String, Integer> commandCounts = new HashMap<>();
    final BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          if (Robot.ROBOT_TYPE != RobotType.REAL)
            Logger.recordOutput(
                "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
                active.booleanValue());
          if (Robot.ROBOT_TYPE != RobotType.REAL)
            Logger.recordOutput("Commands/CommandsAll/" + name, count > 0);
        };

    var scheduler = CommandScheduler.getInstance();

    scheduler.onCommandInitialize(c -> logCommandFunction.accept(c, true));
    scheduler.onCommandFinish(c -> logCommandFunction.accept(c, false));
    scheduler.onCommandInterrupt(c -> logCommandFunction.accept(c, false));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().simulationPeriodic();
      if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput(
            "MapleSim/Pose", swerveDriveSimulation.get().getSimulatedDriveTrainPose());
    }

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Targets/Reef Target", currentTarget);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Targets/Algae Intake Target", algaeIntakeTarget);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Targets/Algae Score Target", algaeScoreTarget);
    Logger.recordOutput(
        "Mechanism Poses",
        new Pose3d[] {
          new Pose3d( // first stage
              new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
          // carriage
          new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d()),
          new Pose3d( // arm
              new Translation3d(
                  ShoulderSubsystem.X_OFFSET_METERS,
                  0,
                  ShoulderSubsystem.Z_OFFSET_METERS + elevator.getExtensionMeters()),
              new Rotation3d(
                  0, -Units.degreesToRadians(2.794042) - shoulder.getAngle().getRadians(), 0.0)),
          new Pose3d( // Manipulator
              new Translation3d(
                      ShoulderSubsystem.X_OFFSET_METERS
                          + shoulder.getAngle().getCos() * ShoulderSubsystem.ARM_LENGTH_METERS,
                      0,
                      elevator.getExtensionMeters()
                          + ShoulderSubsystem.Z_OFFSET_METERS
                          + shoulder.getAngle().getSin() * ShoulderSubsystem.ARM_LENGTH_METERS)
                  .plus(
                      new Translation3d(Units.inchesToMeters(1.0), 0.0, Units.inchesToMeters(-8.0))
                          .rotateBy(new Rotation3d(0.0, -wrist.getAngle().getRadians(), 0.0))),
              new Rotation3d(0, wrist.getAngle().getRadians(), Math.PI))
        });

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "Mechanism Setpoints",
          new Pose3d[] {
            new Pose3d( // first stage
                new Translation3d(0, 0, elevator.getSetpoint() / 2.0), new Rotation3d()),
            // carriage
            new Pose3d(new Translation3d(0, 0, elevator.getSetpoint()), new Rotation3d()),
            new Pose3d( // arm
                new Translation3d(
                    ShoulderSubsystem.X_OFFSET_METERS,
                    0,
                    ShoulderSubsystem.Z_OFFSET_METERS + elevator.getSetpoint()),
                new Rotation3d(
                    0,
                    -Units.degreesToRadians(2.794042) - shoulder.getSetpoint().getRadians(),
                    0.0)),
            new Pose3d( // Manipulator
                new Translation3d(
                        ShoulderSubsystem.X_OFFSET_METERS
                            + shoulder.getSetpoint().getCos() * ShoulderSubsystem.ARM_LENGTH_METERS,
                        0,
                        elevator.getSetpoint()
                            + ShoulderSubsystem.Z_OFFSET_METERS
                            + shoulder.getSetpoint().getSin() * ShoulderSubsystem.ARM_LENGTH_METERS)
                    .plus(
                        new Translation3d(
                                Units.inchesToMeters(1.0), 0.0, Units.inchesToMeters(-8.0))
                            .rotateBy(new Rotation3d(0.0, -wrist.getAngle().getRadians(), 0.0))),
                new Rotation3d(0, wrist.getSetpoint().getRadians(), Math.PI))
          });

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "Shoulder Zero Viz",
          new Pose3d[] {
            new Pose3d( // first stage
                new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
            // carriage
            new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d()),
            new Pose3d( // arm
                new Translation3d(
                    ShoulderSubsystem.X_OFFSET_METERS,
                    0,
                    ShoulderSubsystem.Z_OFFSET_METERS + elevator.getExtensionMeters()),
                new Rotation3d(
                    0,
                    -Units.degreesToRadians(2.794042) - shoulder.getZeroingAngle().getRadians(),
                    0.0)),
            new Pose3d( // Manipulator
                new Translation3d(
                    ShoulderSubsystem.X_OFFSET_METERS
                        + shoulder.getZeroingAngle().getCos() * ShoulderSubsystem.ARM_LENGTH_METERS,
                    0,
                    elevator.getExtensionMeters()
                        + ShoulderSubsystem.Z_OFFSET_METERS
                        + shoulder.getZeroingAngle().getSin()
                            * ShoulderSubsystem.ARM_LENGTH_METERS),
                new Rotation3d(0, wrist.getAngle().getRadians(), Math.PI))
          });
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/CoralTarget",
          Tracer.trace(
              "Get Closest Coral Target", () -> CoralTargets.getClosestTarget(swerve.getPose())));
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "AutoAim/AlgaeIntakeTarget",
          Tracer.trace(
              "Get Closest Algae Target",
              () -> AlgaeIntakeTargets.getClosestTarget(swerve.getPose())));

    carriageLigament.setLength(elevator.getExtensionMeters());
    // Minus 90 to make it relative to horizontal
    shoulderLigament.setAngle(shoulder.getAngle().getDegrees() - 90);
    wristLigament.setAngle(wrist.getAngle().getDegrees() + shoulderLigament.getAngle());
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Mechanism/Elevator", elevatorMech2d);
    superstructure.periodic();
    Logger.recordOutput("Autos/Coral Pre Score", Autos.autoPreScore);
    Logger.recordOutput("Autos/Coral Score", Autos.autoScore);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Autos/Pre Score", Autos.autoPreScore);
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Autos/Score", Autos.autoScore);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "IK/Manipulator FK Pose",
          ExtensionKinematics.getManipulatorPose(
              swerve.getPose(), superstructure.getExtensionState()));
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "IK/Extension FK Pose",
          ExtensionKinematics.solveFK(
              new ExtensionState(
                  elevator.getExtensionMeters(), shoulder.getAngle(), wrist.getAngle())));
    state = superstructure::getState;
  }

  public static void setCurrentCoralTarget(ReefTarget target) {
    currentTarget = target;
  }

  public ReefTarget getCurrentCoralTarget() {
    return currentTarget;
  }

  public static void setCurrentAlgaeIntakeTarget(AlgaeIntakeTarget target) {
    algaeIntakeTarget = target;
  }

  public AlgaeIntakeTarget getCurrentAlgaeIntakeTarget() {
    return algaeIntakeTarget;
  }

  public static void setCurrentAlgaeScoreTarget(AlgaeScoreTarget target) {
    algaeScoreTarget = target;
  }

  public AlgaeScoreTarget getCurrentAlgaeScoreTarget() {
    return algaeScoreTarget;
  }

  public static void setCurrentTarget(ReefTarget target) {
    currentTarget = target;
  }

  public ReefTarget getCurrentTarget() {
    return currentTarget;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
