// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotIOReal;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.swerve.*;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.Tracer;
import frc.robot.utils.autoaim.AutoAim;
import frc.robot.utils.autoaim.AutoAimTargets;
import java.util.HashMap;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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
    COMP(null); // TODO Add swerve constants as appropriate

    public final SwerveConstants swerveConstants;

    private RobotHardware(SwerveConstants swerveConstants) {
      this.swerveConstants = swerveConstants;
    }
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;
  // For replay to work properly this should match the hardware used in the log
  public static final RobotHardware ROBOT_HARDWARE = RobotHardware.ALPHA;

  public static enum ReefTarget {
    L1(0.0),
    L2(ElevatorSubsystem.L2_EXTENSION_METERS),
    L3(ElevatorSubsystem.L3_EXTENSION_METERS),
    L4(ElevatorSubsystem.L4_EXTENSION_METERS);

    public final double elevatorHeight;

    private ReefTarget(double elevatorHeight) {
      this.elevatorHeight = elevatorHeight;
    }
  }

  private ReefTarget currentTarget = ReefTarget.L1;

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
                  new Pose2d(3, 3, new Rotation2d())))
          : Optional.empty();

  private final SwerveSubsystem swerve =
      new SwerveSubsystem(
          ROBOT_HARDWARE.swerveConstants,
          ROBOT_TYPE == RobotType.REAL
              ? new GyroIOPigeon2(ROBOT_HARDWARE.swerveConstants.getGyroID())
              : new GyroIOSim(swerveDriveSimulation.get().getGyroSimulation()),
          // Stream.of(ROBOT_HARDWARE.swerveConstants.getVisionConstants())
          //     .map(
          //         (constants) ->
          //             ROBOT_TYPE == RobotType.REAL
          //                 ? new VisionIOReal(constants)
          //                 : new VisionIOSim(constants))
          //     .toArray(VisionIO[]::new),
          ROBOT_TYPE == RobotType.REAL
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
          swerveDriveSimulation);

  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE == RobotType.REAL ? new ElevatorIOReal() : new ElevatorIOSim());
  private final IntakePivotSubsystem intakePivot =
      new IntakePivotSubsystem(
          ROBOT_TYPE == RobotType.REAL ? new IntakePivotIOReal() : new IntakePivotIOSim());
  private final ManipulatorSubsystem manipulator =
      new ManipulatorSubsystem(
          new RollerIOReal(
              10,
              RollerIOReal.getDefaultConfig()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2))
                  .withSlot0(new Slot0Configs().withKV(0.24).withKP(1.0))),
          new BeambreakIOReal(0, false),
          new BeambreakIOReal(1, false));
  public static final double MANIPULATOR_INDEXING_VELOCITY = 50.0;

  private final Autos autos;
  // Could make this cache like Choreo's AutoChooser, but thats more work and Choreo's default
  // option isn't akit friendly
  // Main benefit to that is reducing startup time, which idt we care about too much
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

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
        // Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
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
    SignalLogger.setPath("/media/sda1/");

    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation.orElse(null));
      swerve.resetPose(swerveDriveSimulation.get().getSimulatedDriveTrainPose());
    }

    autos = new Autos(swerve, manipulator);
    autoChooser.addDefaultOption("None", autos.getNoneAuto());
    autoChooser.addOption("Triangle Test", autos.getTestTriangle());
    autoChooser.addOption("Sprint Test", autos.getTestSprint());
    autoChooser.addOption("Cycle Demo", autos.getDCycle());
    autoChooser.addOption("L4 Auto Run", autos.SLMtoICMD());

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    // Default Commands

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    elevator.setDefaultCommand(elevator.runCurrentZeroing().andThen(elevator.setExtension(0.0)));

    intakePivot.setDefaultCommand(intakePivot.setTargetAngle(IntakePivotSubsystem.RETRACTED_ANGLE));

    manipulator.setDefaultCommand(manipulator.setVelocity(0.0));

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                    modifyJoystick(driver.getLeftY())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    modifyJoystick(driver.getLeftX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    modifyJoystick(driver.getRightX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxAngularSpeed())));

    driver
        .rightBumper()
        .whileTrue(
            AutoAim.translateToPose(
                swerve, () -> AutoAimTargets.getClosestTarget(swerve.getPose())));

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (ROBOT_TYPE == RobotType.SIM) {
                    swerveDriveSimulation.get().setSimulationWorldPose(swerve.getPose());
                  }
                }));

    driver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                elevator.setExtension(() -> currentTarget.elevatorHeight),
                Commands.waitUntil(() -> elevator.isNearExtension(currentTarget.elevatorHeight))
                    .andThen(manipulator.setVelocity(MANIPULATOR_INDEXING_VELOCITY))))
        .onFalse(elevator.setExtension(0.0).until(() -> elevator.isNearExtension(0.0)));

    operator.a().or(driver.a()).onTrue(Commands.runOnce(() -> currentTarget = ReefTarget.L1));
    operator.x().or(driver.x()).onTrue(Commands.runOnce(() -> currentTarget = ReefTarget.L2));
    operator.b().or(driver.b()).onTrue(Commands.runOnce(() -> currentTarget = ReefTarget.L3));
    operator.y().or(driver.y()).onTrue(Commands.runOnce(() -> currentTarget = ReefTarget.L4));

    // Log locations of all autoaim targets
    Logger.recordOutput(
        "AutoAim/Targets",
        Stream.of(AutoAimTargets.values())
            .map((target) -> AutoAimTargets.getRobotTargetLocation(target.location))
            .toArray(Pose2d[]::new));
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
          Logger.recordOutput(
              "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
              active.booleanValue());
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
      Logger.recordOutput(
          "MapleSim/Pose", swerveDriveSimulation.get().getSimulatedDriveTrainPose());
    }

    Logger.recordOutput("Target", currentTarget);
    Logger.recordOutput(
        "Mechanism Poses",
        new Pose3d[] {
          new Pose3d(
              new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
          new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d())
        });
    Logger.recordOutput("AutoAim/Target", AutoAimTargets.getClosestTarget(swerve.getPose()));
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
