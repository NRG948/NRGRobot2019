package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.DriveToVisionTape;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.GearShift;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Gearbox.Gear;
import frc.robot.utilities.Deliver;

public class AutonomousRoutines extends CommandGroup {
  public static final int FIELD_LENGTH_INCHES = 54 * 12;
  public static final int FIELD_WIDTH_INCHES = 27 * 12;

  private static final double DRIVE_POWER = 0.7;
  private static final double TURN_POWER = 1.0;
  private static final double VISION_DELAY = 0.5;

  private AutoMovement autoMovement;
  private AutoStartingPosition autoStartingPosition;
  private AutoFeederPosition autoFeederPosition;
  private AutoMovement autoMovement2;

  /**
   * Read autonomous choosers and build a command group to perform the desired
   * autonomous routine
   */
  public AutonomousRoutines() {
    // addSequential(new SetDriveScale(Drive.SCALE_LOW));

    autoMovement = OI.getAutoMovement();
    autoStartingPosition = OI.getAutoStartingPosition();
    autoFeederPosition = OI.getAutoStationPosition();
    autoMovement2 = OI.getAutoMovement2();
    System.out.println("Auto Movement is : " + autoMovement);
    System.out.println("Auto Position is : " + autoStartingPosition);
    System.out.println("Auto Station position is " + autoFeederPosition);
    System.out.println("Auto Movement 2 is : " + autoMovement2);

    switch (autoMovement) {
    case NONE:
      return;

    case FORWARD:
      addSequential(new DriveStraightDistance(80, DRIVE_POWER), 3);
      return;

    default:
      addSequential(new GearShift(Gear.HIGH));
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoStartingPosition, autoMovement)));
      addSequential(new DelaySeconds(VISION_DELAY));
      addSequential(new DeliverHatch());
      break;
    }

    switch (autoFeederPosition) {
    case NONE:
      addSequential(new GearShift(Gear.LOW));
      return;

    default:
      addSequential(new DriveStraightDistance(6, -DRIVE_POWER));
      addSequential(
          new TurnToHeading((autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER) ? 135 : -135, TURN_POWER));
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoMovement, autoFeederPosition)));
      addSequential(new DelaySeconds(VISION_DELAY));
      addSequential(new PickupHatch());
      addSequential(new DriveStraightDistance(6, -DRIVE_POWER));
      addSequential(new TurnToHeading((autoFeederPosition == AutoFeederPosition.RIGHT_FEEDER) ? 315 : 45, TURN_POWER));
      break;
    }

    switch (autoMovement2) {
    case NONE:
      addSequential(new GearShift(Gear.LOW));
      return;

    default:
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoFeederPosition, autoMovement2)));
      addSequential(new DelaySeconds(VISION_DELAY));
      addSequential(new DeliverHatch());
      addSequential(new DriveStraightDistance(6, -DRIVE_POWER));
      break;
    }
    addSequential(new GearShift(Gear.LOW));
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
