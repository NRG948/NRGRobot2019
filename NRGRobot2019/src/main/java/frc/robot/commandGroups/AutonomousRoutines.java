package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.Robot.HabitatLevel;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.DriveToVisionTape;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.GearShift;
import frc.robot.commands.SetCompressorState;
import frc.robot.commands.TurnToHeading;
import frc.robot.commands.WaitForNewVisionData;
import frc.robot.commands.SetCompressorState.CompressorState;
import frc.robot.subsystems.Gearbox.Gear;
import frc.robot.utilities.VisionTargetsApproach;

public class AutonomousRoutines extends CommandGroup {
  public static final int FIELD_LENGTH_INCHES = 54 * 12;
  public static final int FIELD_WIDTH_INCHES = 27 * 12;

  private static final double DRIVE_POWER = 0.7;
  private static final double TURN_POWER = 1.0;
  private static final double VISION_DELAY = 0.25;

  /**
   * Read autonomous choosers and build a command group to perform the desired
   * autonomous routine
   */
  public AutonomousRoutines() {
    double drivePower = DRIVE_POWER;

    double turnPower = TURN_POWER;
    if (!Robot.drive.areTurnInputsSquared()) {
      turnPower *= turnPower;
    }

    AutoMovement autoMovement = OI.getAutoMovement();
    AutoStartingPosition autoStartingPosition = OI.getAutoStartingPosition();
    AutoFeederPosition autoFeederPosition = OI.getAutoStationPosition();
    AutoMovement autoMovement2 = OI.getAutoMovement2();
    HabitatLevel habLevel = OI.getAutoHabitatLevel();

    System.out.println("Auto Movement is : " + autoMovement);
    System.out.println("Auto Position is : " + autoStartingPosition);
    System.out.println("Auto Station position is " + autoFeederPosition);
    System.out.println("Auto Movement 2 is : " + autoMovement2);
    System.out.println("Auto Habitat level is: " + habLevel);

    addSequential(new GearShift(Gear.HIGH));
    if (habLevel != HabitatLevel.LEVEL_1) {
      addSequential(new DriveStraightDistance(40, 0.7, false));
    }

    switch (autoMovement) {
    case NONE:
      return;

    case FORWARD:
      addSequential(new DriveStraightDistance(80, drivePower), 3);
      return;

    case ROCKET_CLOSE:
      addSequential(new DriveDistanceOnHeading(autoStartingPosition == AutoStartingPosition.RIGHT ? 35.0 : -35.0, 120,
          drivePower));
      addSequential(new WaitForNewVisionData());
      addSequential(new DeliverHatch());
      return;

    case CARGO_FIRST_HATCH_CLOSE:
      addSequential(new DriveStraightDistance(autoStartingPosition == AutoStartingPosition.RIGHT ? 15.0 : -15.0, 170.0,
          drivePower, false));
      addSequential(new TurnToHeading(autoStartingPosition == AutoStartingPosition.RIGHT ? -90.0 : 90.0, turnPower));
      addSequential(new WaitForNewVisionData());
      addSequential(new DeliverHatch());
      return;

    default:
      // addSequential(new
      // FollowPathWeaverFile(getPathWeaverFileName(autoStartingPosition,
      // autoMovement)));
      switch (autoStartingPosition) {
      case LEFT:
        addSequential(new DriveStraightDistance(10.0, 18, drivePower, false));
        addSequential(new DriveStraightDistance(30.0, 50.0, drivePower, false));
        addSequential(new DriveDistanceOnHeading(0.0, 32.0, drivePower * 1.2));
        // addSequential(new DriveDistanceOnHeading(20.0, 106.0, drivePower));
        // addSequential(new TurnToHeading(0, turnPower));
        break;

      case CENTER:
        addSequential(
            new DriveDistanceOnHeading(autoMovement == AutoMovement.CARGO_FRONT_RIGHT_HATCH ? 6 : -6, 90, drivePower));
        break;

      case RIGHT:
        addSequential(new DriveStraightDistance(-10.0, 18, drivePower, false));
        addSequential(new DriveStraightDistance(-30.0, 50.0, drivePower, false));
        addSequential(new DriveDistanceOnHeading(0.0, 32.0, drivePower * 1.2));
        // addSequential(new DriveDistanceOnHeading(-20.0, 106.0, drivePower));
        // addSequential(new TurnToHeading(0, turnPower));
        break;
      }
      addSequential(new WaitForNewVisionData());
      addSequential(new DeliverHatch());
      break;
    }

    switch (autoFeederPosition) {
    case NONE:
      return;

    case RIGHT_FEEDER:
      // addSequential(new SetCompressorState(CompressorState.DISABLED));
      addSequential(new DriveStraightDistance(-60.0, 125.0, -1.0, false));
      addSequential(new TurnToHeading(-180, turnPower));
      addSequential(new DriveDistanceOnHeading(-180.0, 44.0, drivePower*0.9));
      // addSequential(new SetCompressorState(CompressorState.ENABLED));
      // addSequential(new DelaySeconds(4));
      addSequential(new WaitForNewVisionData());
      addSequential(new PickupHatch());
      // addSequential(new DriveStraightDistance(6, -drivePower));
      break;

    default:
      // addSequential(new DriveStraightDistance(6, -drivePower));
      addSequential(new TurnToHeading((autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER) ? 135 : -135, turnPower));
      System.out.println("Gyro heading " + RobotMap.navx.getAngle());
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoMovement, autoFeederPosition)));
      addSequential(new WaitForNewVisionData());
      addSequential(new PickupHatch());
      addSequential(new DriveStraightDistance(6, -drivePower));
      addSequential(new TurnToHeading((autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER) ? 315 : 45, turnPower));
      break;
    }

    switch (autoMovement2) {
    case NONE:
      return;
    
    case CARGO_FIRST_HATCH_CLOSE:
      addSequential(new DriveDistanceOnHeading(autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER ? -190 : 190, 240, -1.0));
      addSequential(new TurnToHeading(autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER ? -85 : 85, turnPower));
      addSequential(new WaitForNewVisionData());
      addSequential(new DeliverHatch());
      break;

    default:
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoFeederPosition, autoMovement2)));
      addSequential(new WaitForNewVisionData());
      addSequential(new DeliverHatch());
      // addSequential(new DriveStraightDistance(6, -drivePower));
      break;
    }
  }

  private String getPathWeaverFileName(AutoMovement from, AutoFeederPosition to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  private String getPathWeaverFileName(AutoStartingPosition from, AutoMovement to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  private String getPathWeaverFileName(AutoFeederPosition from, AutoMovement to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  private String getPathWeaverFileName(String from, String to) {
    return "output/" + from + "_TO_" + to + ".pf1.csv";
  }
}
