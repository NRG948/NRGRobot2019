package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.commandGroups.AutonomousRoutines;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.PullForwardUntilOnHab;
import frc.robot.commands.SetClimberHeight;
import frc.robot.commands.SetRobotRoll;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoAcquirer;
import frc.robot.subsystems.ClimberArmWheels;
import frc.robot.subsystems.ClimberRear;
import frc.robot.subsystems.ClimberPistons;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gearbox;
import frc.robot.subsystems.HatchClawSubsystem;
import frc.robot.subsystems.HatchExtensionSubsystem;
import frc.robot.subsystems.ClimberPistons.State;
import frc.robot.utilities.NRGPreferences;
import frc.robot.utilities.PositionTracker;
import frc.robot.utilities.VisionTargets;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final String DEFAULT_TEST_PATH = "LEFT_TO_CARGO_FRONT_LEFT_HATCH";

  public static OI oi;

  public static Gearbox gearbox;
  public static Drive drive;
  public static CargoAcquirer cargoAcquirer;
  public static Arm arm;
  public static ClimberRear climberRear;
  public static ClimberArmWheels climberArmWheels;
  public static ClimberPistons climberPistons;
  public static HatchClawSubsystem hatchClaw;
  public static HatchExtensionSubsystem hatchExtension;

  public static PositionTracker positionTracker = new PositionTracker();
  // public static PowerDistributionPanel pdp = new PowerDistributionPanel();

  // public static Watchdog watchdog = new Watchdog(0.02, () -> {
  // });

  Command autonomousCommand;
  public static SendableChooser<AutoStartingPosition> autoStartingPositionChooser;
  public static SendableChooser<AutoMovement> autoMovementChooser;
  public static VisionTargets visionTargets;
  public static SendableChooser<AutoFeederPosition> autoStationPositionChooser;
  public static SendableChooser<AutoMovement> autoMovement2Chooser;
  public static SendableChooser<HabitatLevel> habLevelChooser;

  public enum AutoStartingPosition {
    LEFT, CENTER, RIGHT
  }

  public enum AutoFeederPosition {
    NONE, LEFT_FEEDER, RIGHT_FEEDER
  }

  public enum AutoMovement {
    NONE, FORWARD, CARGO_FRONT_LEFT_HATCH, CARGO_FRONT_RIGHT_HATCH, ROCKET_CLOSE, CARGO_FIRST_HATCH_CLOSE
  }

  public enum HabitatLevel {
    LEVEL_1, LEVEL_2
  }

  public static Boolean isPracticeBot() {
    return NRGPreferences.BooleanPrefs.USING_PRACTICE_BOT.getValue();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("robotInit()");


    RobotMap.init();
    NRGPreferences.init();

    // initialize subsystems
    drive = new Drive();
    gearbox = new Gearbox();
    arm = new Arm();
    climberPistons = new ClimberPistons();
    climberRear = new ClimberRear();
    cargoAcquirer = new CargoAcquirer();
    climberArmWheels = new ClimberArmWheels();
    hatchClaw = new HatchClawSubsystem();
    hatchExtension = new HatchExtensionSubsystem();

    oi = new OI();
    visionTargets = new VisionTargets();

    autoStartingPositionChooser = new SendableChooser<AutoStartingPosition>();
    autoStartingPositionChooser.setDefaultOption("Left", AutoStartingPosition.LEFT);
    autoStartingPositionChooser.addOption("Center", AutoStartingPosition.CENTER);
    autoStartingPositionChooser.addOption("Right", AutoStartingPosition.RIGHT);

    autoStationPositionChooser = new SendableChooser<AutoFeederPosition>();
    autoStationPositionChooser.setDefaultOption("None", AutoFeederPosition.NONE);
    autoStationPositionChooser.addOption("Left", AutoFeederPosition.LEFT_FEEDER);
    autoStationPositionChooser.addOption("Right", AutoFeederPosition.RIGHT_FEEDER);

    autoMovementChooser = new SendableChooser<AutoMovement>();
    autoMovementChooser.setDefaultOption("None", AutoMovement.NONE);
    autoMovementChooser.addOption("Forward", AutoMovement.FORWARD);
    autoMovementChooser.addOption("Cargo_front_left_hatch", AutoMovement.CARGO_FRONT_LEFT_HATCH);
    autoMovementChooser.addOption("Cargo_front_right_hatch", AutoMovement.CARGO_FRONT_RIGHT_HATCH);
    autoMovementChooser.addOption("Cargo_first_hatch_close", AutoMovement.CARGO_FIRST_HATCH_CLOSE);
    autoMovementChooser.addOption("Rocket_close", AutoMovement.ROCKET_CLOSE);

    autoMovement2Chooser = new SendableChooser<AutoMovement>();
    autoMovement2Chooser.setDefaultOption("None", AutoMovement.NONE);
    autoMovement2Chooser.addOption("Forward", AutoMovement.FORWARD);
    autoMovement2Chooser.addOption("Cargo_front_left_hatch", AutoMovement.CARGO_FRONT_LEFT_HATCH);
    autoMovement2Chooser.addOption("Cargo_front_right_hatch", AutoMovement.CARGO_FRONT_RIGHT_HATCH);
    autoMovement2Chooser.addOption("Cargo_first_hatch_close", AutoMovement.CARGO_FIRST_HATCH_CLOSE);
    autoMovement2Chooser.addOption("Rocket_close", AutoMovement.ROCKET_CLOSE);

    habLevelChooser = new SendableChooser<HabitatLevel>();
    habLevelChooser.setDefaultOption("Level 1", HabitatLevel.LEVEL_1);
    habLevelChooser.addOption("Level 2", HabitatLevel.LEVEL_2);

    // Shuffleboard.getTab("Power").add(Robot.pdp).withPosition(0, 0).withSize(3,
    // 3);

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add("Start", autoStartingPositionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 0)
        .withSize(4, 1);
    autoTab.add("First Hatch", autoMovementChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 1)
        .withSize(6, 1);
    autoTab.add("Feeder", autoStationPositionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 2)
        .withSize(4, 1);
    autoTab.add("End", autoMovement2Chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 3)
        .withSize(6, 1);
    autoTab.add("Habitat Level", habLevelChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 0)
        .withSize(2, 1);

    arm.initShuffleboard();

    ShuffleboardTab testTab = Shuffleboard.getTab("Test");
    testTab.add("Test Path", (new InstantCommand(() -> {
      String pathname = NRGPreferences.StringPrefs.TEST_PATH_NAME.getValue();
      new FollowPathWeaverFile("output/" + pathname + ".pf1.csv").start();
    }))).withSize(2, 1).withPosition(0, 0);
    ShuffleboardLayout distanceButtonLayout = testTab.getLayout("Test Distance", BuiltInLayouts.kList)
        .withPosition(0, 1).withSize(2, 2);
    distanceButtonLayout.add("12 Inches", new DriveDistanceOnHeading(0, 12, 0.7));
    distanceButtonLayout.add("24 Inches", new DriveDistanceOnHeading(0, 24, 0.7));
    distanceButtonLayout.add("48 Inches", new DriveDistanceOnHeading(0, 48, 0.7));

    ShuffleboardLayout climbButtonLayout = testTab.getLayout("Test Climb", BuiltInLayouts.kList)
        .withPosition(2, 0).withSize(2, 4);
    climbButtonLayout.add("Set Pitch 1", new SetRobotRoll(1.0));
    climbButtonLayout.add("Set Pitch 8", new SetRobotRoll(8.0));
    climbButtonLayout.add("Drive Until On Hab", new PullForwardUntilOnHab());
    climbButtonLayout.add("Set Climber Height", new SetClimberHeight(1115));
    climbButtonLayout.add("Set Climber Height To 0", new SetClimberHeight(0));
    testTab.add("Position Tracker", positionTracker).withSize(2, 3).withPosition(4, 0);
    climberPistons.setState(State.RETRACT);
    System.out.println("robotInit() done");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("PositionTracker/x", positionTracker.getX());
    SmartDashboard.putNumber("PositionTracker/y", positionTracker.getY());
    SmartDashboard.putNumber("PositionTracker/maxVelocity", positionTracker.getMaxVelocity());
    SmartDashboard.putData("LeftEncoder", RobotMap.driveLeftEncoder);
    SmartDashboard.putData("RightEncoder", RobotMap.driveRightEncoder);
    SmartDashboard.putNumber("Gyro", RobotMap.navx.getAngle());
    SmartDashboard.putData("DriveSubsystem", Robot.drive);

    SmartDashboard.putBoolean("Vision/cameraInverted", Robot.arm.isCameraInverted());
    boolean hasTargets = visionTargets.hasTargets();
    SmartDashboard.putBoolean("Vision/hasTargets", hasTargets);
    if (hasTargets) {
      SmartDashboard.putNumber("Vision/angleToTarget", visionTargets.getAngleToTarget());
      SmartDashboard.putNumber("Vision/distance", visionTargets.getDistanceToTarget());
      Point center = visionTargets.getCenterOfTargets();
      SmartDashboard.putNumber("Vision/centerX", center.x);
      SmartDashboard.putNumber("Vision/centerY", center.y);
    }

    SmartDashboard.putBoolean("Hatch/DefenseOK", Robot.hatchClaw.isOpen() && Robot.hatchExtension.isRetracted());
    SmartDashboard.putNumber("Trigger/Right", Robot.oi.xboxController.getRawAxis(2));
    SmartDashboard.putNumber("Trigger/Left", Robot.oi.xboxController.getRawAxis(3));
    SmartDashboard.putNumber("IRSensor", RobotMap.IRSensor.getAverageVoltage());
    SmartDashboard.putNumber("Climber Encoder", RobotMap.climberRearEncoder.getDistance());
    SmartDashboard.putNumber("Roll", RobotMap.navx.getRoll());
    SmartDashboard.putNumber("Pitch", RobotMap.navx.getPitch());
  }

  /**
   * This function is called once each time the robot e%nters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    RobotMap.cameraLights.set(Value.kOff);
  }

  @Override
  public void disabledPeriodic() {
    visionTargets.update();
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit()");
    RobotMap.cameraLights.set(Value.kForward);
    RobotMap.resetSensors();
    Robot.arm.armAnglePIDInit();

    autonomousCommand = new AutonomousRoutines();
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // watchdog.reset();
    positionTracker.updatePosition();
    // watchdog.addEpoch("position tracker");
    Robot.arm.armAnglePIDExecute();
    // watchdog.addEpoch("arm angle PID");
    Scheduler.getInstance().run();
    // watchdog.addEpoch("scheduler");
    // if (watchdog.isExpired()) {
    //   watchdog.printEpochs();
    // }
  }

  @Override
  public void teleopInit() {
    System.out.println("teleopInit()");
    RobotMap.cameraLights.set(Value.kForward);
    Robot.arm.armAnglePIDInit();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // watchdog.reset();
    positionTracker.updatePosition();
    // watchdog.addEpoch("position tracker");
    Robot.arm.armAnglePIDExecute();
    // watchdog.addEpoch("arm angle PID");
    Scheduler.getInstance().run();
    // watchdog.addEpoch("scheduler");
    // if (watchdog.isExpired()) {
    //   watchdog.printEpochs();
    // }
  }

  @Override
  public void testPeriodic() {
    positionTracker.updatePosition();
    visionTargets.update();
  }
}